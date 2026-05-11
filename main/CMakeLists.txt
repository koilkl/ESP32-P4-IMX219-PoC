idf_component_register(SRCS
                    "main.cpp"
                    "jpeg_enc.c"
                    "../../TFLite/model_settings.cpp"
                    "../../TFLite/person_detect_model_data.cpp"
                    INCLUDE_DIRS "." "../../TFLite"
                    REQUIRES arduino esp_video esp_cam_sensor esp-tflite-micro driver imx219 esp_timer esp_wifi esp_http_server esp_driver_jpeg nvs_flash esp_netif)
