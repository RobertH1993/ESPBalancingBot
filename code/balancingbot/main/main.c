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
#include "kallman.h"

// Measured gyro X bias when the IMU is laying flat on the table
#define GYRO_X_BIAS 1.4f
// How much of the new speed to use compared to the old speed, this works as a low pass filter
#define SPEED_FILTER_ALPHA 0.3f
// How many cm the wheel travels per encoder tick
#define WHEEL_CM_PER_ENCODER_TICK 0.00571428571
// Maximum angle before the robot stops
#define MAX_ANGLE_BEFORE_STOP 42.0f
// How close to upright (deg) the robot must be held before it starts balancing
#define STEADY_ANGLE_THRESHOLD 0.25f

#define ONE_SECOND_IN_MICROSECONDS 1000000.0f;

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

// Bundles all the state the control loop carries between iterations
typedef struct{
    qmi8658_dev_t imu;
    qmi8658_data_t imu_data;
    kallman_filter_t kf;
    float speed_filtered;
    uint8_t counter;
    int64_t last_time;
    char pid_data[UDP_MAX_PACKET_SIZE];
} control_ctx_t;

// Immediately stop both motors
static inline void stop_motors(void){
    wheel_set_speed(LEFT_WHEEL, 0);
    wheel_set_speed(RIGHT_WHEEL, 0);
}

// Accelerometer-derived pitch angle in degrees
static inline float compute_pitch(const qmi8658_data_t *imu_data){
    return atan2(imu_data->accelY, -imu_data->accelZ) * 180.0 / M_PI;
}

// One-time IMU bring-up and calibration
static void imu_setup(qmi8658_dev_t *imu){
    rstate.mode = ROBOT_STATE_INIT;

    ESP_ERROR_CHECK(qmi8658_init(imu, master_bus, QMI8658_I2C_ADDRESS_H));
    qmi8658_set_accel_unit_mps2(imu, true);
    qmi8658_set_accel_range(imu, QMI8658_ACCEL_RANGE_2G);
    qmi8658_set_gyro_range(imu, QMI8658_GYRO_RANGE_256DPS);

    // Calibrate sensor
    rstate.mode = ROBOT_STATE_CALIBRATING;
    ESP_ERROR_CHECK(qmi8658_enable_accel(imu, false));
    ESP_ERROR_CHECK(qmi8658_enable_gyro(imu, false));
    ESP_ERROR_CHECK(qmi8658_write_register(imu, QMI8658_CTRL9, 0xA2));
    ESP_LOGI("MAIN", "Calibrating IMU, hold steady");
    vTaskDelay(pdMS_TO_TICKS(3000));
    uint8_t status = 1;
    qmi8658_read_register(imu, QMI8658_COD_STATUS, &status, 1);
    ESP_LOGW("MAIN", "COD Status: %u", status);

    // Turn on sensors
    ESP_ERROR_CHECK(qmi8658_enable_accel(imu, true));
    ESP_ERROR_CHECK(qmi8658_enable_gyro(imu, true));
}

// Reset the filter, PIDs and odometry so we start balancing from a clean state
static void enter_balancing(control_ctx_t *ctx, float pitch){
    kallman_init(&ctx->kf, pitch);
    ctx->speed_filtered = 0.0f;
    ctx->counter = 0;

    pid_reset(&rstate.pids[PID_BALANCE], 0.0f);
    pid_reset(&rstate.pids[PID_SPEED], 0.0f);
    pid_reset(&rstate.pids[PID_WHEEL_TRIM], 0.0f);


    rstate.distance_left = 0.0f;
    rstate.distance_right = 0.0f;
    wheel_reset_encoder_count(LEFT_WHEEL);
    wheel_reset_encoder_count(RIGHT_WHEEL);

    ESP_LOGI("MAIN", "Settled, start balancing");
    rstate.mode = ROBOT_STATE_BALANCING;
}

// SETTLING: keep motors off until the robot is held upright and steady
static uint8_t settling_counter = 0;
static void handle_settling(control_ctx_t *ctx, float pitch){
    stop_motors();

    if(fabsf(pitch) <= STEADY_ANGLE_THRESHOLD){
        settling_counter++;
        if(settling_counter >= 3){
            enter_balancing(ctx, pitch);
            settling_counter = 0;
        }
    }else{
        settling_counter = 0;
    }
}

// BALANCING: run the full cascade of speed, balance and trim PIDs
static void handle_balancing(control_ctx_t *ctx, float pitch, float dt){
    float filtered_angle_x_kallman = kallman_update(&ctx->kf, pitch, -ctx->imu_data.gyroX - GYRO_X_BIAS, dt);

    // If the pitch is too high, the robot fell over
    if(fabsf(filtered_angle_x_kallman) >= MAX_ANGLE_BEFORE_STOP){
        ESP_LOGE("MAIN", "Pitch is too high, robot fell over");
        stop_motors();
        rstate.mode = ROBOT_STATE_FALLEN;
        return;
    }

    // Calculate speed and distance
    float speed_left = wheel_get_encoder_pulses(LEFT_WHEEL, true) * WHEEL_CM_PER_ENCODER_TICK / dt; //CM/s
    float speed_right = wheel_get_encoder_pulses(RIGHT_WHEEL, true) * WHEEL_CM_PER_ENCODER_TICK / dt; //CM/s
    rstate.distance_left += speed_left;
    rstate.distance_right += speed_right;

    // Filter the speed to prevent quantization noise
    ctx->speed_filtered = SPEED_FILTER_ALPHA * ((speed_left + speed_right) / 2.0f) + (1.0f - SPEED_FILTER_ALPHA) * ctx->speed_filtered;


    if(ctx->counter == 9){ 
        rstate.pids[PID_SPEED].setpoint = rstate.target_speed;
        float new_setpoint = pid_compute(&rstate.pids[PID_SPEED], ctx->speed_filtered, dt * 10.0f, 0.0f);
        
        float difference = new_setpoint - rstate.pids[PID_BALANCE].setpoint;
        float alpha = 0.25f;

        if(fabsf(difference) >= 5.0f){
            alpha = 0.5f;
        }
        rstate.pids[PID_BALANCE].setpoint = rstate.pids[PID_BALANCE].setpoint * alpha + new_setpoint * (1.0f - alpha);
        ctx->counter = 0;
    }
    ctx->counter++;

    // Calculate the balance
    float pid_output = pid_compute(&rstate.pids[PID_BALANCE], filtered_angle_x_kallman, dt, (ctx->imu_data.gyroX - GYRO_X_BIAS));

    // Calculate wheel trim for straight driving + apply turn rate
    // Positive turn_rate = turn right (right wheel slower, left wheel faster)
    rstate.pids[PID_WHEEL_TRIM].setpoint = rstate.target_turn_rate;
    float wheel_trim = pid_compute(&rstate.pids[PID_WHEEL_TRIM], (rstate.distance_left - rstate.distance_right), dt, 0.0f);


    float pwm_output = pid_output;
    wheel_set_speed(LEFT_WHEEL, pwm_output - wheel_trim);
    wheel_set_speed(RIGHT_WHEEL, pwm_output + wheel_trim);

    // Send telemetry
    //int len = snprintf(ctx->pid_data, UDP_MAX_PACKET_SIZE, "speed:%f\nb_setpoint:%f\ns_p:%f\ns_i:%f\n",
    //    ctx->speed_filtered,
    //    rstate.pids[PID_BALANCE].setpoint,
    //    rstate.pids[PID_SPEED].P,
    //    rstate.pids[PID_SPEED].I
    //);

    //tnc_push_data(ctx->pid_data, len);
}

// FALLEN: motors stay off until the robot is set upright again, then
// go back through SETTLING to recover without a reboot
static void handle_fallen(float pitch){
    stop_motors();

    if(fabsf(pitch) <= STEADY_ANGLE_THRESHOLD){
        ESP_LOGI("MAIN", "Back upright, settling before balancing again");
        rstate.mode = ROBOT_STATE_SETTLING;
    }
}

void control_task(void* pvParams){
    control_ctx_t ctx = {0};

    imu_setup(&ctx.imu);

    // Task timing
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    wheel_reset_encoder_count(LEFT_WHEEL);
    wheel_reset_encoder_count(RIGHT_WHEEL);

    ctx.last_time = esp_timer_get_time();

    // Start by waiting for the robot to be held upright and steady
    rstate.mode = ROBOT_STATE_SETTLING;

    while(true){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        //Calculate DT
        int64_t now = esp_timer_get_time();
        float dt = (float)(now - ctx.last_time) / ONE_SECOND_IN_MICROSECONDS;
        ctx.last_time = now;

        // Calculate the current pitch
        if(qmi8658_read_sensor_data(&ctx.imu, &ctx.imu_data) != ESP_OK) continue;
        float pitch = compute_pitch(&ctx.imu_data);

        switch(rstate.mode){
            case ROBOT_STATE_SETTLING:  handle_settling(&ctx, pitch);   break;
            case ROBOT_STATE_BALANCING: handle_balancing(&ctx, pitch, dt); break;
            case ROBOT_STATE_FALLEN:    handle_fallen(pitch);           break;
            default: break;
        }
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
            .ssid = "KPN900536_2g",
            .password = "REDACTED",

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
 
    rstate.distance_left = 0.0f;
    rstate.distance_right = 0.0f;
    rstate.target_speed = 0.0f;
    rstate.target_turn_rate = 0.0f;

    // Balance pid
    rstate.pids[PID_BALANCE].Kp = -75.0f;
    rstate.pids[PID_BALANCE].Ki = 0.0f;
    rstate.pids[PID_BALANCE].Kd = -1.0f;
    rstate.pids[PID_BALANCE].setpoint = 0.0f;
    rstate.pids[PID_BALANCE].max_output = 1000.0f;

    // Speed pid
    rstate.pids[PID_SPEED].Kp = -0.7f;
    rstate.pids[PID_SPEED].Ki = -0.25f;
    rstate.pids[PID_SPEED].Kd = +0.02f;
    rstate.pids[PID_SPEED].setpoint = +0.0f;
    rstate.pids[PID_SPEED].max_output = 35.0f;

    // Wheel trim pid
    rstate.pids[PID_WHEEL_TRIM].Kp = 0.8f;
    rstate.pids[PID_WHEEL_TRIM].Ki = 0.1f;
    rstate.pids[PID_WHEEL_TRIM].Kd = 0.0f;
    rstate.pids[PID_WHEEL_TRIM].setpoint = 0.0f;
    rstate.pids[PID_WHEEL_TRIM].max_output = 150.0f;


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
