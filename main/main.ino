#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "esp_log.h"
#include "imx219.h"
#include "esp_video_init.h"
#include "esp_video_ioctl.h"
#include "esp_video_device.h"
#include "esp_cam_sensor_detect.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "jpeg_enc.h"

static const char *TAG = "app_main";

static const gpio_num_t I2C_MASTER_SCL_IO = GPIO_NUM_8;
static const gpio_num_t I2C_MASTER_SDA_IO = GPIO_NUM_7;
static const int I2C_MASTER_NUM = 0;
static const int I2C_MASTER_FREQ_HZ = 100000;
static const gpio_num_t XCLK_PIN = GPIO_NUM_20;
static const int XCLK_FREQ_HZ = 24000000;

#define IMG_WIDTH  1536
#define IMG_HEIGHT 1232
#define OUT_WIDTH  96
#define OUT_HEIGHT 96

static int *s_x_lut = NULL;
static int *s_y_lut = NULL;
static int fd = -1;
static void *mapped_bufs[2] = {0};
static uint8_t *rgb_buf = NULL;
static uint8_t *gray_buf = NULL;
static uint64_t last_time = 0;
static uint64_t last_frame_send = 0;
static uint32_t frame_count = 0;
static uint32_t sent_count = 0;

#define FRAME_INTERVAL_US 100000  // 10 FPS = 100ms per frame

static void enable_xclk(void) {
    ledc_timer_config_t ledc_timer;
    memset(&ledc_timer, 0, sizeof(ledc_timer));
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution = LEDC_TIMER_1_BIT;
    ledc_timer.freq_hz = XCLK_FREQ_HZ;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel;
    memset(&ledc_channel, 0, sizeof(ledc_channel));
    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.duty = 1;
    ledc_channel.gpio_num = XCLK_PIN;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.hpoint = 0;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&ledc_channel);
    ESP_LOGI(TAG, "XCLK Enabled");
}

static void init_demosaic_luts(int width, int height) {
    s_x_lut = (int *)malloc(OUT_WIDTH * sizeof(int));
    s_y_lut = (int *)malloc(OUT_HEIGHT * sizeof(int));
    
    int crop_w = 1232;
    int crop_h = 1232;
    int x_offset = (width - crop_w) / 2;
    int y_offset = (height - crop_h) / 2;
    
    float x_step = (float)crop_w / OUT_WIDTH;
    float y_step = (float)crop_h / OUT_HEIGHT;
    
    for (int y = 0; y < OUT_HEIGHT; y++) {
        s_y_lut[y] = (y_offset + (int)(y * y_step + 0.5f)) & ~1;
    }
    for (int x = 0; x < OUT_WIDTH; x++) {
        s_x_lut[x] = (x_offset + (int)(x * x_step + 0.5f)) & ~1;
    }
    
    ESP_LOGI(TAG, "Sensor: %dx%d, Center crop: %dx%d at offset (%d,%d), Output: %dx%d",
             width, height, crop_w, crop_h, x_offset, y_offset, OUT_WIDTH, OUT_HEIGHT);
}

static void demosaic_bggr_to_rgb(const uint8_t *raw10, uint8_t *rgb, int width, int height) {
    for (int y = 0; y < OUT_HEIGHT; y++) {
        int src_y = s_y_lut[y];
        int row0 = src_y * (width * 5 / 4);
        int row1 = (src_y + 1) * (width * 5 / 4);
        int out_row = y * OUT_WIDTH;
        for (int x = 0; x < OUT_WIDTH; x++) {
            int src_x = s_x_lut[x];
            int col = (src_x >> 2) * 5 + (src_x % 4);
            uint8_t b = raw10[row0 + col];
            uint8_t g = (raw10[row0 + col + 1] + raw10[row1 + col]) >> 1;
            uint8_t r = raw10[row1 + col + 1];
            int out_idx = (out_row + x) * 3;
            rgb[out_idx + 0] = r;
            rgb[out_idx + 1] = g;
            rgb[out_idx + 2] = b;
        }
    }
}

static void rgb_to_gray(const uint8_t *rgb, uint8_t *gray, int pixel_count) {
    for (int i = 0; i < pixel_count; i++) {
        int idx = i * 3;
        uint8_t r = rgb[idx + 0];
        uint8_t g = rgb[idx + 1];
        uint8_t b = rgb[idx + 2];
        gray[i] = (uint8_t)(((uint16_t)r * 30 + (uint16_t)g * 59 + (uint16_t)b * 11) / 100);
    }
}

const uint8_t syncHeader[] = {0xAA, 0x55, 0xAA};

void setup() {
    Serial0.begin(921600);
    delay(100);

    nvs_flash_init();
    init_demosaic_luts(IMG_WIDTH, IMG_HEIGHT);
    jpeg_enc_init(OUT_WIDTH, OUT_HEIGHT);
    enable_xclk();
    delay(100);
    imx219_force_link();

    esp_video_init_csi_config_t csi_config;
    memset(&csi_config, 0, sizeof(csi_config));
    csi_config.sccb_config.init_sccb = true;
    csi_config.sccb_config.i2c_config.port = I2C_MASTER_NUM;
    csi_config.sccb_config.i2c_config.scl_pin = I2C_MASTER_SCL_IO;
    csi_config.sccb_config.i2c_config.sda_pin = I2C_MASTER_SDA_IO;
    csi_config.sccb_config.freq = I2C_MASTER_FREQ_HZ;
    csi_config.reset_pin = GPIO_NUM_NC;
    csi_config.pwdn_pin = GPIO_NUM_NC;

    esp_video_init_config_t cam_config;
    memset(&cam_config, 0, sizeof(cam_config));
    cam_config.csi = &csi_config;
    esp_video_init(&cam_config);

    fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open CSI device: %s", strerror(errno));
        return;
    }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = IMG_WIDTH;
    fmt.fmt.pix.height = IMG_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR10;
    ioctl(fd, VIDIOC_S_FMT, &fmt);

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 2;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ioctl(fd, VIDIOC_REQBUFS, &req);

    for (int i = 0; i < 2; i++) {
        struct v4l2_buffer b;
        memset(&b, 0, sizeof(b));
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        ioctl(fd, VIDIOC_QUERYBUF, &b);
        mapped_bufs[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        ioctl(fd, VIDIOC_QBUF, &b);
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMON, &type);

    rgb_buf = (uint8_t *)heap_caps_malloc(OUT_WIDTH * OUT_HEIGHT * 3, MALLOC_CAP_SPIRAM);
    gray_buf = (uint8_t *)heap_caps_malloc(OUT_WIDTH * OUT_HEIGHT, MALLOC_CAP_SPIRAM);
    last_time = esp_timer_get_time();
    ESP_LOGI(TAG, "Camera setup complete. Sending 96x96 grayscale images...");
    ESP_LOGI(TAG, "rgb_buf=%p, gray_buf=%p", rgb_buf, gray_buf);
}

void loop() {
    if (fd < 0 || rgb_buf == NULL) {
        delay(1000);
        return;
    }

    uint64_t now = esp_timer_get_time();
    if ((now - last_time) >= 1000000) {
        ESP_LOGI(TAG, "Capture FPS: %lu, Send FPS: %lu", 
                 (unsigned long)frame_count, (unsigned long)sent_count);
        frame_count = 0;
        sent_count = 0;
        last_time = now;
    }

    struct v4l2_buffer buf_dq;
    memset(&buf_dq, 0, sizeof(buf_dq));
    buf_dq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf_dq.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_DQBUF, &buf_dq) == 0) {
        frame_count++;
        uint8_t *raw_data = (uint8_t *)mapped_bufs[buf_dq.index];
        
        if (now - last_frame_send >= FRAME_INTERVAL_US) {
            demosaic_bggr_to_rgb(raw_data, rgb_buf, IMG_WIDTH, IMG_HEIGHT);
            rgb_to_gray(rgb_buf, gray_buf, OUT_WIDTH * OUT_HEIGHT);

            esp_log_level_t prev_level = esp_log_level_get("*");
            esp_log_level_set("*", ESP_LOG_NONE);
            
            fwrite(syncHeader, 1, 3, stdout);
            fwrite(gray_buf, 1, OUT_WIDTH * OUT_HEIGHT, stdout);
            fflush(stdout);
            
            esp_log_level_set("*", prev_level);

            last_frame_send = now;
            sent_count++;
        }

        ioctl(fd, VIDIOC_QBUF, &buf_dq);
    }

    delay(1);
}
