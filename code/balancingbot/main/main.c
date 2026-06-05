/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "driver/i2c_master.h"

#include "telemetryandcontrol.h"
#include "wheel.h"
#include "state.h"
#include "board.h"

#include "qmi8658.h"

#include <math.h>

// QMI accel https://components.espressif.com/components/waveshare/qmi8658/versions/1.0.1/readme
// INA power sensor https://components.espressif.com/components/esp-idf-lib/ina219/versions/1.0.7/readme


robot_state_t rstate = {0};
i2c_master_bus_handle_t master_bus = NULL;

// Minimal event handler for wifi events
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("MAIN", "Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect(); // Retry connection on error
        ESP_LOGW("MAIN", "Connection lost, retry");
    }
}

void control_task(void* pvParams){

    qmi8658_dev_t imu = {0};
    qmi8658_data_t imu_data = {0};
    ESP_ERROR_CHECK(qmi8658_init(&imu, master_bus, QMI8658_I2C_ADDRESS_H));
    qmi8658_set_accel_unit_mps2(&imu, true);

    // Calibrate sensor
    ESP_ERROR_CHECK(qmi8658_enable_accel(&imu, false));
    ESP_ERROR_CHECK(qmi8658_enable_gyro(&imu, false));
    ESP_ERROR_CHECK(qmi8658_write_register(&imu, QMI8658_CTRL9, 0xA2));
    ESP_LOGI("MAIN", "Calibrating IMU, hold steady");
    vTaskDelay(pdMS_TO_TICKS(3000));
    uint8_t status = 1;
    qmi8658_read_register(&imu, QMI8658_COD_STATUS, &status, 1);
    ESP_LOGW("MAIN", "COD Status: %u", status);

    // Turn on sensors
    ESP_ERROR_CHECK(qmi8658_enable_accel(&imu, true));
    ESP_ERROR_CHECK(qmi8658_enable_gyro(&imu, true));
    ESP_ERROR_CHECK(qmi8658_write_register(&imu, QMI8658_CTRL5, 0xFF));


    int64_t last_time = esp_timer_get_time();

    // Task timing
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    char pid_data[UDP_MAX_PACKET_SIZE] = {0};

    // Start out with a measurement to not build up error when started
    float filtered_angle_x = 0.0f;
    float alpha = 0.02;
    uint8_t counter = 0;

    wheel_reset_encoder_count(LEFT_WHEEL);
    wheel_reset_encoder_count(RIGHT_WHEEL);

    // Let angle settle before movement
    float start_angle = 100.0f;
    while(fabsf(start_angle) > 0.25f){
        ESP_ERROR_CHECK(qmi8658_read_sensor_data(&imu, &imu_data));
        start_angle = atan2(imu_data.accelY, -imu_data.accelZ) * 180.0 / M_PI;
        vTaskDelay(pdTICKS_TO_MS(10));
    }
    filtered_angle_x = start_angle;


    while(true){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        //Calculate DT
        int64_t now = esp_timer_get_time();
        float dt = (float)(now - last_time) / 1000000.0f;
        last_time = now;

        // Calculate the current pitch
        if(qmi8658_read_sensor_data(&imu, &imu_data) != ESP_OK) continue;
        float pitch = atan2(imu_data.accelY, -imu_data.accelZ) * 180.0 / M_PI;
        filtered_angle_x = (alpha * pitch) + (1.0f - alpha) * (filtered_angle_x + (-(imu_data.gyroX - 1.4f)* dt));


        // Calculate speed and distance
        float speed_left = wheel_get_encoder_pulses(LEFT_WHEEL, true) * 0.00571428571 * (1/dt); //CM/s
        float speed_right = wheel_get_encoder_pulses(RIGHT_WHEEL, true) * 0.00571428571 * (1/dt); //CM/s
        rstate.distance_left += speed_left;
        rstate.distance_right += speed_right;

        // Switch between holding position and driving
        if(rstate.pids[PID_SPEED].setpoint != 0.0f && rstate.status == HOLD_POSITION) rstate.status = DRIVE;
        else if(rstate.pids[PID_SPEED].setpoint == 0.0f && rstate.status == DRIVE){
            rstate.pids[PID_POSITION].setpoint = rstate.distance_left;
            rstate.pids[PID_POSITION].no_d_counter = 2;
            rstate.status = HOLD_POSITION;
        }

        if(counter == 10 && rstate.status == HOLD_POSITION){ // Calculate position pid
            rstate.pids[PID_BALANCE].setpoint = pid_compute(&rstate.pids[PID_POSITION], rstate.distance_left, dt);
            counter = 0;
        }else if(counter == 10 && rstate.status == DRIVE){ // Calculate speed pid
            rstate.pids[PID_BALANCE].setpoint = pid_compute(&rstate.pids[PID_SPEED], (speed_left + speed_right) / 2, dt);
            counter = 0;
        }else{
            counter++;
        }

        // Calculate the balance
        float pid_output = pid_compute(&rstate.pids[PID_BALANCE], filtered_angle_x, dt);
        if(fabsf(rstate.pids[PID_BALANCE].prev_error) > 10.0f){
            pid_output *= 1.25f; //Increase power above 10 degrees
        }

        // Calculate wheel trim so the robot keeps driving straight
        float wheel_trim = pid_compute(&rstate.pids[PID_WHEEL_TRIM], (rstate.distance_left - rstate.distance_right), dt);

        float pwm_output = pid_output;
        wheel_set_speed(LEFT_WHEEL, pwm_output - wheel_trim);
        wheel_set_speed(RIGHT_WHEEL, pwm_output + wheel_trim);
        
        // Send telemetry
        int len = snprintf(pid_data, UDP_MAX_PACKET_SIZE, "LeftD:%f\nRightD:%f\ntrim:%f\n", 
            rstate.distance_left,
            rstate.distance_right,
            wheel_trim
        );

        tnc_push_data(pid_data, len);
    }
}


void app_main(void)
{
    // Init NVS storage
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_LOGW("MAIN", "Erased NVS because it was truncated!");
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI("MAIN", "Initialized NVS!");

    // Init the TCP/IP Stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Init WiFi
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

    // Register wifi event callbacks
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    // 5. Hardcoded credentials
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",

            // Force WPA2
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            // Faster connection
            .scan_method = WIFI_FAST_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL, // Always connect to strongest AP

            .pmf_cfg = {
                .capable = true,
                .required = false
            }
        },
    };

    // Connect
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI("MAIN", "WiFi started waiting for connection....");

    // Init wheels
    wheel_init_hardware();

    // Init I2C Bus
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .sda_io_num = I2C_MASTER_0_SDA_IO,
        .scl_io_num = I2C_MASTER_0_SCL_IO,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &master_bus));

    // Start telemetry and control
    tnc_cfg_t tnc_config = {0};
    tnc_config.udp_port = 3334;
    tnc_start(&tnc_config);

    vTaskDelay(pdMS_TO_TICKS(5000));

    rstate.status = HOLD_POSITION;

    // Balance pid
    rstate.pids[PID_BALANCE].Kp = -100.0f;
    rstate.pids[PID_BALANCE].Ki = -15.0f;
    rstate.pids[PID_BALANCE].Kd = -1.0f;
    rstate.pids[PID_BALANCE].setpoint = 3.19f;
    rstate.pids[PID_BALANCE].max_output = 850.0f;
    rstate.pids[PID_BALANCE].no_d_counter = 40;

    // Position pid
    rstate.pids[PID_POSITION].Kp = -0.002f;
    rstate.pids[PID_POSITION].Ki = -0.00001f;
    rstate.pids[PID_POSITION].Kd = -0.00033f;
    rstate.pids[PID_POSITION].setpoint = 0.0f;
    rstate.pids[PID_POSITION].max_output = 17.5f;
    rstate.pids[PID_POSITION].no_d_counter = 15;

    // Speed pid
    rstate.pids[PID_SPEED].Kp = -0.3f;
    rstate.pids[PID_SPEED].Ki = 0.0f;
    rstate.pids[PID_SPEED].Kd = -0.00001f;
    rstate.pids[PID_SPEED].setpoint = 0.0f;
    rstate.pids[PID_SPEED].max_output = 10.0f;

    // Wheel trim pid
    rstate.pids[PID_WHEEL_TRIM].Kp = 0.8f;
    rstate.pids[PID_WHEEL_TRIM].Ki = 0.0f;
    rstate.pids[PID_WHEEL_TRIM].Kd = 0.0f;
    rstate.pids[PID_WHEEL_TRIM].setpoint = 0.0f;
    rstate.pids[PID_WHEEL_TRIM].max_output = 20.0f;


    xTaskCreatePinnedToCore(
        control_task,
        "pid_control_task",
        4096,
        NULL,
        configMAX_PRIORITIES -1,
        NULL,
        1
    );
}
