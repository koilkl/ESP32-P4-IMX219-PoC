#include <Arduino.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "esp_log.h"
#include "esp_err.h"
#include "imx219.h"
#include "esp_video_init.h"
#include "esp_video_ioctl.h"
#include "esp_video_device.h"
#include "esp_cam_sensor_detect.h"
#include "esp_cam_sensor_xclk.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "jpeg_enc.h"

static const char *TAG = "app_main";

static const gpio_num_t I2C_MASTER_SCL_IO = GPIO_NUM_26;//8-26
static const gpio_num_t I2C_MASTER_SDA_IO = GPIO_NUM_27;//7-27
static const int I2C_MASTER_NUM = 0;
static const int I2C_MASTER_FREQ_HZ = 100000;
static const gpio_num_t XCLK_PIN = GPIO_NUM_20;
static constexpr uint32_t kXclkScanHz[] = {24000000, 19200000, 12000000, 6000000};
static constexpr size_t kXclkScanIndex = 0;
static_assert(kXclkScanIndex < (sizeof(kXclkScanHz) / sizeof(kXclkScanHz[0])), "kXclkScanIndex out of range");
static constexpr int32_t kHsSettleOverride = -1;
static constexpr int32_t kLineSyncOverride = -1;
static constexpr uint32_t kExperimentObserveSeconds = 10;

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
static volatile bool s_in_dqbuf = false;
static volatile uint64_t s_dqbuf_enter_us = 0;
static esp_cam_sensor_format_t s_forced_sensor_fmt;
static constexpr bool kForceSensorFmt = false;
static TaskHandle_t s_capture_task = NULL;
static TaskHandle_t s_fps_task = NULL;
static esp_cam_sensor_xclk_handle_t s_xclk_handle = NULL;
static uint64_t s_capture_start_us = 0;
static bool s_experiment_summary_logged = false;

#define FRAME_INTERVAL_US 100000  // 10 FPS = 100ms per frame

static uint32_t active_xclk_hz() {
    return kXclkScanHz[kXclkScanIndex];
}

static bool sensor_patch_requested() {
    return kHsSettleOverride >= 0 || kLineSyncOverride >= 0;
}

static void log_scan_configuration() {
    ESP_LOGI(TAG, "scan config: xclk_idx=%u xclk_hz=%lu hs_settle_override=%ld line_sync_override=%ld force_sensor_fmt=%d observe_s=%lu",
             (unsigned)kXclkScanIndex, (unsigned long)active_xclk_hz(), (long)kHsSettleOverride,
             (long)kLineSyncOverride, (int)kForceSensorFmt, (unsigned long)kExperimentObserveSeconds);
    ESP_LOGI(TAG, "xclk candidates: [0]=24000000 [1]=19200000 [2]=12000000 [3]=6000000");
}

static void log_sensor_fmt(const char *stage, const esp_cam_sensor_format_t *fmt) {
    ESP_LOGI(TAG, "sensor fmt(%s): w=%u h=%u out_fmt=%u port=%u xclk=%d mipi_clk=%lu lanes=%lu hs_settle=%lu line_sync=%d",
             stage, (unsigned)fmt->width, (unsigned)fmt->height, (unsigned)fmt->format, (unsigned)fmt->port,
             (int)fmt->xclk, (unsigned long)fmt->mipi_info.mipi_clk, (unsigned long)fmt->mipi_info.lane_num,
             (unsigned long)fmt->mipi_info.hs_settle, (int)fmt->mipi_info.line_sync_en);
}

static void warn_if_sensor_mode_changed(const esp_cam_sensor_format_t *before, const esp_cam_sensor_format_t *after) {
    if (after->width == 1920 && after->height == 1080) {
        ESP_LOGW(TAG, "sensor fmt jumped to 1080p; stop this scan point and avoid mixing it with 1536x1232 RAW10 capture");
    }
    if (before->width != after->width || before->height != after->height || before->format != after->format) {
        ESP_LOGW(TAG, "sensor mode changed: before=%ux%u fmt=%u after=%ux%u fmt=%u",
                 (unsigned)before->width, (unsigned)before->height, (unsigned)before->format,
                 (unsigned)after->width, (unsigned)after->height, (unsigned)after->format);
    }
}

static void log_v4l2_capture_fmt(const char *stage, const struct v4l2_format *fmt) {
    ESP_LOGI(TAG, "capture fmt(%s): w=%u h=%u fourcc=%c%c%c%c field=%u bytesperline=%u sizeimage=%u",
             stage, (unsigned)fmt->fmt.pix.width, (unsigned)fmt->fmt.pix.height,
             fmt->fmt.pix.pixelformat & 0xff, (fmt->fmt.pix.pixelformat >> 8) & 0xff,
             (fmt->fmt.pix.pixelformat >> 16) & 0xff, (fmt->fmt.pix.pixelformat >> 24) & 0xff,
             (unsigned)fmt->fmt.pix.field, (unsigned)fmt->fmt.pix.bytesperline, (unsigned)fmt->fmt.pix.sizeimage);
}

static void log_experiment_summary_if_due(uint64_t now_us, uint32_t total_frame, uint32_t total_send, uint64_t dq_ms) {
    if (s_experiment_summary_logged || s_capture_start_us == 0) {
        return;
    }
    if ((now_us - s_capture_start_us) < (uint64_t)kExperimentObserveSeconds * 1000000ULL) {
        return;
    }
    s_experiment_summary_logged = true;
    ESP_LOGI(TAG, "SCAN SUMMARY: xclk_idx=%u xclk_hz=%lu hs_settle_override=%ld line_sync_override=%ld observe_s=%lu total_frame=%lu total_send=%lu dq_ms=%llu result=%s",
             (unsigned)kXclkScanIndex, (unsigned long)active_xclk_hz(), (long)kHsSettleOverride,
             (long)kLineSyncOverride, (unsigned long)kExperimentObserveSeconds,
             (unsigned long)total_frame, (unsigned long)total_send, (unsigned long long)dq_ms,
             total_frame > 0 ? "FRAME_OK" : "NO_FRAME");
}

static void enable_xclk(void) {
    if (s_xclk_handle != NULL) {
        esp_cam_sensor_xclk_stop(s_xclk_handle);
        esp_cam_sensor_xclk_free(s_xclk_handle);
        s_xclk_handle = NULL;
    }

    esp_err_t err = esp_cam_sensor_xclk_allocate(ESP_CAM_SENSOR_XCLK_ESP_CLOCK_ROUTER, &s_xclk_handle);
    ESP_LOGI(TAG, "xclk_allocate: %s", esp_err_to_name(err));
    if (err != ESP_OK) {
        return;
    }

    esp_cam_sensor_xclk_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.esp_clock_router_cfg.xclk_pin = XCLK_PIN;
    cfg.esp_clock_router_cfg.xclk_freq_hz = active_xclk_hz();
    err = esp_cam_sensor_xclk_start(s_xclk_handle, &cfg);
    ESP_LOGI(TAG, "xclk_start: %s gpio=%d freq=%lu", esp_err_to_name(err), (int)XCLK_PIN, (unsigned long)active_xclk_hz());
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

static void inspect_and_patch_sensor_fmt_if_needed() {
    if (fd < 0) {
        return;
    }

    esp_cam_sensor_format_t before;
    memset(&before, 0, sizeof(before));
    if (ioctl(fd, VIDIOC_G_SENSOR_FMT, &before) == 0) {
        log_sensor_fmt("before", &before);
    } else {
        ESP_LOGW(TAG, "VIDIOC_G_SENSOR_FMT failed: errno=%d (%s)", errno, strerror(errno));
        return;
    }

    if (!kForceSensorFmt && !sensor_patch_requested()) {
        return;
    }

    memcpy(&s_forced_sensor_fmt, &before, sizeof(s_forced_sensor_fmt));
    if (kForceSensorFmt) {
        s_forced_sensor_fmt.name = "forced";
        s_forced_sensor_fmt.width = IMG_WIDTH;
        s_forced_sensor_fmt.height = IMG_HEIGHT;
        s_forced_sensor_fmt.format = ESP_CAM_SENSOR_PIXFORMAT_RAW10;
        s_forced_sensor_fmt.port = ESP_CAM_SENSOR_MIPI_CSI;
        s_forced_sensor_fmt.xclk = active_xclk_hz();
        if (s_forced_sensor_fmt.mipi_info.mipi_clk == 0) {
            s_forced_sensor_fmt.mipi_info.mipi_clk = 456000000;
        }
        if (s_forced_sensor_fmt.mipi_info.lane_num == 0) {
            s_forced_sensor_fmt.mipi_info.lane_num = 2;
        }
    } else {
        s_forced_sensor_fmt.name = "patched";
    }
    if (kHsSettleOverride >= 0) {
        s_forced_sensor_fmt.mipi_info.hs_settle = (uint32_t)kHsSettleOverride;
    }
    if (kLineSyncOverride >= 0) {
        s_forced_sensor_fmt.mipi_info.line_sync_en = kLineSyncOverride ? 1 : 0;
    }
    ESP_LOGI(TAG, "request sensor fmt(%s): xclk=%d mipi_clk=%lu lanes=%lu hs_settle=%lu line_sync=%d",
             kForceSensorFmt ? "force" : "patch",
             (int)s_forced_sensor_fmt.xclk, (unsigned long)s_forced_sensor_fmt.mipi_info.mipi_clk,
             (unsigned long)s_forced_sensor_fmt.mipi_info.lane_num, (unsigned long)s_forced_sensor_fmt.mipi_info.hs_settle,
             (int)s_forced_sensor_fmt.mipi_info.line_sync_en);
    if (ioctl(fd, VIDIOC_S_SENSOR_FMT, &s_forced_sensor_fmt) == 0) {
        ESP_LOGI(TAG, "VIDIOC_S_SENSOR_FMT OK");
    } else {
        ESP_LOGW(TAG, "VIDIOC_S_SENSOR_FMT failed: errno=%d (%s)", errno, strerror(errno));
    }

    esp_cam_sensor_format_t after;
    memset(&after, 0, sizeof(after));
    if (ioctl(fd, VIDIOC_G_SENSOR_FMT, &after) == 0) {
        log_sensor_fmt("after", &after);
        warn_if_sensor_mode_changed(&before, &after);
    }
}

static void fps_task(void *arg) {
    (void)arg;
    uint32_t last_frame = 0;
    uint32_t last_sent = 0;
    while (true) {
        uint64_t now = esp_timer_get_time();
        uint64_t dq_ms = 0;
        if (s_in_dqbuf && s_dqbuf_enter_us != 0) {
            dq_ms = (now - s_dqbuf_enter_us) / 1000;
        }
        uint32_t cur_frame = frame_count;
        uint32_t cur_sent = sent_count;
        uint32_t d_frame = cur_frame - last_frame;
        uint32_t d_sent = cur_sent - last_sent;
        last_frame = cur_frame;
        last_sent = cur_sent;
        ESP_LOGI(TAG, "Capture FPS: %lu, Send FPS: %lu (in_dqbuf=%d dq_ms=%llu total_frame=%lu total_send=%lu)",
                 (unsigned long)d_frame, (unsigned long)d_sent, (int)s_in_dqbuf,
                 (unsigned long long)dq_ms, (unsigned long)cur_frame, (unsigned long)cur_sent);
        log_experiment_summary_if_due(now, cur_frame, cur_sent, dq_ms);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void capture_task(void *arg) {
    (void)arg;
    while (true) {
        if (fd < 0 || rgb_buf == NULL) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        uint64_t now = esp_timer_get_time();
        struct v4l2_buffer buf_dq;
        memset(&buf_dq, 0, sizeof(buf_dq));
        buf_dq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf_dq.memory = V4L2_MEMORY_MMAP;

        s_in_dqbuf = true;
        s_dqbuf_enter_us = esp_timer_get_time();
        int dq_ret = ioctl(fd, VIDIOC_DQBUF, &buf_dq);
        s_in_dqbuf = false;
        s_dqbuf_enter_us = 0;
        if (dq_ret == 0) {
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
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void setup() {
    Serial0.begin(921600);
    delay(100);

    nvs_flash_init();
    init_demosaic_luts(IMG_WIDTH, IMG_HEIGHT);
    jpeg_enc_init(OUT_WIDTH, OUT_HEIGHT);
    log_scan_configuration();
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

    inspect_and_patch_sensor_fmt_if_needed();

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = IMG_WIDTH;
    fmt.fmt.pix.height = IMG_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR10;
    log_v4l2_capture_fmt("request", &fmt);
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "VIDIOC_S_FMT failed: errno=%d (%s)", errno, strerror(errno));
        return;
    }
    log_v4l2_capture_fmt("applied", &fmt);

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 2;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed: errno=%d (%s)", errno, strerror(errno));
        return;
    }
    ESP_LOGI(TAG, "VIDIOC_REQBUFS OK: count=%u", (unsigned)req.count);

    for (int i = 0; i < 2; i++) {
        struct v4l2_buffer b;
        memset(&b, 0, sizeof(b));
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = i;
        if (ioctl(fd, VIDIOC_QUERYBUF, &b) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QUERYBUF failed index=%d: errno=%d (%s)", i, errno, strerror(errno));
            return;
        }
        mapped_bufs[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        if (mapped_bufs[i] == MAP_FAILED) {
            mapped_bufs[i] = NULL;
            ESP_LOGE(TAG, "mmap failed index=%d: errno=%d (%s)", i, errno, strerror(errno));
            return;
        }
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF failed index=%d: errno=%d (%s)", i, errno, strerror(errno));
            return;
        }
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "VIDIOC_STREAMON failed: errno=%d (%s)", errno, strerror(errno));
        return;
    }
    ESP_LOGI(TAG, "VIDIOC_STREAMON OK");

    rgb_buf = (uint8_t *)heap_caps_malloc(OUT_WIDTH * OUT_HEIGHT * 3, MALLOC_CAP_SPIRAM);
    gray_buf = (uint8_t *)heap_caps_malloc(OUT_WIDTH * OUT_HEIGHT, MALLOC_CAP_SPIRAM);
    if (rgb_buf == NULL || gray_buf == NULL) {
        ESP_LOGE(TAG, "frame buffers allocation failed: rgb=%p gray=%p", rgb_buf, gray_buf);
        return;
    }
    last_time = esp_timer_get_time();
    s_capture_start_us = last_time;
    s_experiment_summary_logged = false;
    ESP_LOGI(TAG, "Camera setup complete. Sending 96x96 grayscale images...");
    ESP_LOGI(TAG, "rgb_buf=%p, gray_buf=%p", rgb_buf, gray_buf);

    xTaskCreate(fps_task, "fps", 4096, NULL, 1, &s_fps_task);
    xTaskCreate(capture_task, "cap", 8192, NULL, 2, &s_capture_task);
}

void loop() {
    delay(1000);
}
