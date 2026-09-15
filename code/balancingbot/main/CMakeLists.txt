idf_component_register(SRCS "esp_hid_gap.c" "bt_control.c" "pid.c" "wheel.c" "main.c" "telemetryandcontrol.c" "wheel.c" "pid.c" "qmi8658.c" "kallman.c"
                       PRIV_REQUIRES spi_flash nvs_flash esp_wifi mdns esp_driver_pcnt esp_driver_gpio esp_driver_mcpwm esp_timer esp_driver_i2c proto bt esp_hid
                       INCLUDE_DIRS "")
