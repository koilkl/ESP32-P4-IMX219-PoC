#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
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
// #include <linux/videodev2.h>

static const char *TAG = "app_main";

#define I2C_MASTER_SCL_IO           8
#define I2C_MASTER_SDA_IO           7
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          100000
#define XCLK_PIN                    20
#define XCLK_FREQ_HZ                24000000

#define IMG_WIDTH  1536
#define IMG_HEIGHT 1232
#define OUT_WIDTH  800
#define OUT_HEIGHT 600

static int *s_x_lut = NULL;
static int *s_y_lut = NULL;

// --- 1. 基础硬件配置函数 ---
static void enable_xclk(void) {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_1_BIT,
        .freq_hz = XCLK_FREQ_HZ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 1,
        .gpio_num   = XCLK_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0,
    };
    ledc_channel_config(&ledc_channel);
    ESP_LOGI(TAG, "XCLK Enabled");
}

// --- 2. 图像处理函数 ---
static void init_demosaic_luts(int width, int height) {
    s_x_lut = malloc(OUT_WIDTH * sizeof(int));
    s_y_lut = malloc(OUT_HEIGHT * sizeof(int));
    for (int y = 0; y < OUT_HEIGHT; y++) s_y_lut[y] = ((y * 192) / 100) & ~1;
    for (int x = 0; x < OUT_WIDTH; x++) s_x_lut[x] = ((x * 192) / 100) & ~1;
}

static void demosaic_bggr_to_rgb(const uint8_t *raw10, uint8_t *rgb, int width, int height) {
    for (int y = 0; y < OUT_HEIGHT; y++) {
        int src_y = s_y_lut[y];
        int row0 = src_y * (width * 5 / 4);
        int row1 = (src_y + 1) * (width * 5 / 4);
        int out_row = (OUT_HEIGHT - 1 - y) * OUT_WIDTH;
        for (int x = 0; x < OUT_WIDTH; x++) {
            int src_x = s_x_lut[x];
            int col = (src_x >> 2) * 5 + (src_x % 4);
            uint8_t b = raw10[row0 + col];
            uint8_t g = (raw10[row0 + col + 1] + raw10[row1 + col]) >> 1;
            uint8_t r = raw10[row1 + col + 1];
            int out_idx = (out_row + (OUT_WIDTH - 1 - x)) * 3;
            rgb[out_idx + 0] = r; rgb[out_idx + 1] = g; rgb[out_idx + 2] = b;
        }
    }
}

// --- 3. 串口导出函数 ---
static void export_image_to_console(uint8_t *data, size_t len) {
    if (len < 4 || data[0] != 0xFF || data[1] != 0xD8) {
        return;
    }

    esp_log_level_t prev_level = esp_log_level_get("*");
    esp_log_level_set("*", ESP_LOG_NONE);

    printf("\n---JPEG_START:%zu---\n", len);
    uint32_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        printf("%02X", data[i]);
        checksum += data[i];
        if (i % 128 == 0) {
            vTaskDelay(1); 
        }
    }
    printf("\n---CSUM:%u---\n", (unsigned int)checksum);
    printf("---JPEG_END---\n");

    esp_log_level_set("*", prev_level);
}

void app_main(void) {
    nvs_flash_init();
    init_demosaic_luts(IMG_WIDTH, IMG_HEIGHT);
    jpeg_enc_init(OUT_WIDTH, OUT_HEIGHT);
    enable_xclk();
    vTaskDelay(pdMS_TO_TICKS(100));
    imx219_force_link();

    esp_video_init_csi_config_t csi_config = {
        .sccb_config = { .init_sccb = true, 
            .i2c_config = { .port = I2C_MASTER_NUM, .scl_pin = I2C_MASTER_SCL_IO, .sda_pin = I2C_MASTER_SDA_IO },
            .freq = I2C_MASTER_FREQ_HZ },
        .reset_pin = -1, .pwdn_pin = -1,
    };
    esp_video_init_config_t cam_config = { .csi = &csi_config };
    esp_video_init(&cam_config);

    int fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR);
    
    struct v4l2_format fmt = {0}; 
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = IMG_WIDTH;
    fmt.fmt.pix.height = IMG_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR10;
    ioctl(fd, VIDIOC_S_FMT, &fmt);

    struct v4l2_requestbuffers req = { .count = 2, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
    ioctl(fd, VIDIOC_REQBUFS, &req);

    void *mapped_bufs[2];
    for (int i = 0; i < 2; i++) {
        struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = i };
        ioctl(fd, VIDIOC_QUERYBUF, &b);
        mapped_bufs[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        ioctl(fd, VIDIOC_QBUF, &b);
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMON, &type);

    uint8_t *rgb_buf = heap_caps_malloc(OUT_WIDTH * OUT_HEIGHT * 3, MALLOC_CAP_SPIRAM);
    
    // --- 修正：声明缺失的变量 ---
    uint64_t last_export = 0;
    uint64_t last_time = esp_timer_get_time();
    uint32_t frame_count = 0;

    while (1) {
        uint64_t now = esp_timer_get_time();

        // FPS 统计逻辑：移出导出逻辑，确保 frame_count 被正确处理
        if ((now - last_time) >= 1000000) {
            ESP_LOGI(TAG, "FPS: %lu", (unsigned long)frame_count);
            frame_count = 0;
            last_time = now;
        }

        struct v4l2_buffer buf_dq = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
        if (ioctl(fd, VIDIOC_DQBUF, &buf_dq) == 0) {
            frame_count++; // 增加帧计数
            uint8_t *raw_data = (uint8_t*)mapped_bufs[buf_dq.index];
            if (rgb_buf) {
                demosaic_bggr_to_rgb(raw_data, rgb_buf, IMG_WIDTH, IMG_HEIGHT);
                
                uint8_t *jpg_ptr = NULL; 
                int jpg_len = 0;
                
                // 只有在获取到 jpg_ptr 后才进行导出
                if (jpeg_enc_process(rgb_buf, OUT_WIDTH*OUT_HEIGHT*3, OUT_WIDTH, OUT_HEIGHT, V4L2_PIX_FMT_RGB24, &jpg_ptr, &jpg_len) == ESP_OK) {
                    if (now - last_export > 5000000) {
                        export_image_to_console(jpg_ptr, jpg_len);
                        last_export = now;
                        last_time = now; // 导出后重置计时以避开 FPS 日志
                    }
                    free(jpg_ptr); // 释放内存
                }
            }
            ioctl(fd, VIDIOC_QBUF, &buf_dq);
        }
        vTaskDelay(1);
    }
}