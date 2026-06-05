idf_component_register(SRCS "pid.c" "wheel.c" "main.c" "telemetryandcontrol.c" "wheel.c" "pid.c" "qmi8658.c"
                       PRIV_REQUIRES spi_flash nvs_flash esp_wifi mdns esp_driver_pcnt esp_driver_gpio esp_driver_mcpwm esp_timer esp_driver_i2c proto
                       INCLUDE_DIRS "")
