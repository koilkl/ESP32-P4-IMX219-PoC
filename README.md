# ESP32-P4-IMX219-PoC
Experimental Proof of Concept enabling Sony IMX219 (RPi Camera v2) color streaming on ESP32-P4 via MIPI CSI-2 and software demosaicing (V4L2).

![Status](https://img.shields.io/badge/Status-Experimental-orange)
![Chip](https://img.shields.io/badge/Chip-ESP32--P4-red)
![Camera](https://img.shields.io/badge/Sensor-Sony_IMX219-blue)
![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.5.4-blue)

This project is a **Proof of Concept (PoC)** demonstrating how to interface a **Sony IMX219 (Raspberry Pi Camera v2)** with the **ESP32-P4** using the MIPI CSI-2 interface to capture and process color images.

Since the official ISP (Image Signal Processor) driver for the ESP32-P4 is not yet fully available for public use, this project implements a **hybrid software/hardware pipeline** to bypass the limitation and achieve color image output.

## 🚀 Key Features

* **V4L2 Implementation:** Uses the standard `video4linux2` API via `esp_video` component for camera control and frame capture
* **RAW10 Capture:** Captures raw Bayer data (SBGGR10 format) directly from the MIPI sensor at 1536x1232 resolution
* **Software Demosaicing:** Implements a custom CPU-based bilinear interpolation (`demosaic_bggr_to_rgb`) to convert RAW10 Bayer data to RGB888 format
* **Hardware JPEG Encoding:** Feeds the RGB data into the ESP32-P4's internal **JPEG Encoder** for efficient image compression
* **PSRAM Memory Optimization:** Uses external SPI RAM for large frame buffers to save internal memory for other operations
* **Serial Export:** Exports compressed JPEG images over UART serial port for easy integration with host systems

## ⚡ Performance

* **Input Resolution:** 1536x1232 (RAW10 Bayer format)
* **Output Resolution:** 800x600 (RGB888 to JPEG compressed)
* **Framerate:** ~8 FPS (limited by single-core software demosaicing)
* **Memory Usage:** ~3.6MB for RGB frame buffer (allocated in PSRAM)
* **Output Format:** Standard JPEG image (YUV422 compressed)

## 🔌 Hardware Requirements

### Supported Boards
* Any ESP32-P4 development board with MIPI CSI-2 connector and PSRAM (minimum 8MB PSRAM recommended)
* Tested on: Espressif ESP32-P4 Function EV Board

### Wiring
This configuration uses the standard **15-pin FPC MIPI** connector layout found on most ESP32-P4 development boards:

| Pin Function | ESP32-P4 GPIO | Description |
| :--- | :--- | :--- |
| **I2C SDA** | `GPIO 7` | SCCB control data for sensor configuration |
| **I2C SCL** | `GPIO 8` | SCCB control clock for sensor configuration |
| **XCLK** | `GPIO 20` | 24MHz Master Clock (Generated via LEDC peripheral) |
| **MIPI CSI-2 Lanes** | *Internal* | Uses internal MIPI PHY (2-lane configuration) |
| **Power** | 3.3V | 3.3V power supply for camera module |

### Camera Module
* Sony IMX219 (Raspberry Pi Camera v2) or compatible MIPI CSI-2 camera module
* 15-pin FPC flat cable (0.5mm pitch)

## 🛠️ Software Dependencies

* **ESP-IDF:** v5.5.4 (stable release recommended for ESP32-P4 support)
* **Required Components:**
    * `esp_video` (V4L2 compatible video capture framework)
    * `esp_cam_sensor` (camera sensor driver framework)
    * `driver` (LEDC, I2C, JPEG encoder drivers)
    * `heap_caps` (for PSRAM memory allocation)

## ⚙️ Key Configuration (sdkconfig)

The project requires specific ESP-IDF configuration options to be enabled:

### PSRAM Configuration (Critical)
```ini
CONFIG_SPIRAM=y                          # Enable external SPI RAM
CONFIG_SPIRAM_MODE_HEX=y                 # Hexadecimal mode for PSRAM
CONFIG_SPIRAM_SPEED_200M=y               # 200MHz PSRAM clock speed
CONFIG_SPIRAM_USE_MALLOC=y               # Use malloc for PSRAM allocation
CONFIG_SPIRAM_BOOT_INIT=y                # Initialize PSRAM during boot
CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y  # Allow stack in external memory
CONFIG_FATFS_ALLOC_PREFER_EXTRAM=y       # Prefer external RAM for FATFS
CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM=y  # Allow task stacks in external RAM
```

### Other Important Configurations
```ini
CONFIG_ISP_CTRL_FUNC_IN_IRAM=y           # ISP control functions in IRAM for performance
CONFIG_ESP_SLEEP_PSRAM_LEAKAGE_WORKAROUND=y  # PSRAM leakage workaround for sleep mode
CONFIG_IDF_INIT_VERSION="5.5.4"          # ESP-IDF version compatibility
```

## 🏗️ Build & Flash

### 1. Clone the repository
```bash
git clone https://github.com/yourusername/ESP32-P4-IMX219-PoC.git
cd ESP32-P4-IMX219-PoC
```

### 2. Set target and configure (optional)
```bash
idf.py set-target esp32p4
# Use menuconfig to customize settings if needed
# idf.py menuconfig
```

### 3. Build and flash
```bash
idf.py build flash monitor
```

### 4. Retrieve Images
The device will export JPEG images over the serial port every 5 seconds. Look for output in the format:
```
---JPEG_START:12345---
FFD8FFE000104A46494600010100000100010000FFDB00430003020203020203030303040303040508...
---CSUM:1234567---
---JPEG_END---
```

You can use the provided TMConnector tool to automatically capture and save these images to your computer.

## 📐 Architecture Overview

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  IMX219 Sensor  │────▶│  MIPI CSI-2     │────▶│  RAW10 Buffer   │
│  (1536x1232)    │     │  Interface      │     │  (Internal RAM) │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                                                         │
                                                         ▼
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  JPEG Output    │◀────│  Hardware JPEG  │◀────│  Software       │
│  (800x600)      │     │  Encoder        │     │  Demosaicing    │
└─────────────────┘     └─────────────────┘     │  (RGB Buffer in │
                                                 │  PSRAM)         │
                                                 └─────────────────┘
```

## ⚠️ Technical Notes & Limitations

1. **Software Demosaicing:** The Bayer to RGB conversion is currently performed on the CPU, which is computationally expensive and limits the framerate to ~8 FPS. Once Espressif releases the full ISP driver, this step can be offloaded to hardware for 30FPS+ performance.

2. **Image Quality:** Since there is no hardware ISP processing (Auto White Balance, Auto Exposure, Gamma Correction, Noise Reduction), the image relies on the raw sensor output and might appear dark or green-tinted depending on lighting conditions.

3. **PSRAM Requirement:** The project requires PSRAM to store the large RGB frame buffer (800x600x3 = ~1.44MB). Without PSRAM enabled, the application will crash due to insufficient memory.

4. **Serial Export Only:** Current version exports images over UART serial port. HTTP streaming support will be added in a future release.

5. **Resolution:** The current implementation uses 1536x1232 input resolution for optimal balance between image quality and performance. This can be adjusted in `main.c` if needed.

## 🤝 Troubleshooting

### Common Issues:
1. **Camera not detected:** Check wiring, especially I2C pins and XCLK signal. Ensure the camera module is properly connected.
2. **Memory allocation failed:** Verify that PSRAM is enabled in sdkconfig. Check PSRAM size (minimum 2MB required, 8MB recommended).
3. **Low FPS:** This is expected with software demosaicing. Try reducing the output resolution in `main.c` for higher framerates.
4. **Image artifacts:** Check MIPI lane connections and ensure the camera module is properly seated in the FPC connector.

## 📜 License

MIT License. See LICENSE file for details.

## 🤝 Acknowledgments

* **Hardware & Testing:** Developed and tested for ESP32-P4 platform
* **ESP-IDF Framework:** Thanks to Espressif for providing the ESP-IDF framework and MIPI CSI-2 support
* **Community:** Thanks to the ESP32 community for continuous research on ESP32-P4 capabilities
