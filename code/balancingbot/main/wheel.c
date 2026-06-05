#include "wheel.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/pulse_cnt.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "board.h"

static uint8_t control_initialized = 0;
mcpwm_timer_handle_t wheel_motor_timer = NULL;

// Timer config, resolution between 0 - 1000
const static mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 20000000, // 20MHz
        .period_ticks = 1000,      // 1kHz PWM frequentie
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
};
// This needs to be the same as period_ticks
#define MOTOR_MAX_PWM_VALUE 1000

typedef struct{
    // GPIO to control the H-bridge forward / backward
    gpio_num_t pin_1;
    gpio_num_t pin_2;

    // PWM pin to control the speed of the motor
    gpio_num_t pwm_pin;

    // Handles to the MCPWM components
    mcpwm_oper_handle_t operator;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;

    uint32_t max_pwm_value;

} motor_t;

typedef struct{
    motor_t motor;
    pcnt_unit_handle_t encoder;
} wheel_motor_t;


wheel_motor_t wheels[2] = {0};

// Private
motor_t init_wheel_motor(gpio_num_t pin1, gpio_num_t pin2, gpio_num_t pwm_pin){
    motor_t new_motor;
    new_motor.pin_1 = pin1;
    new_motor.pin_2 = pin2;
    new_motor.pwm_pin = pwm_pin;

    // H-Bridge control pins
    gpio_set_direction(pin1, GPIO_MODE_OUTPUT);
    gpio_set_level(pin1, 0);
    gpio_set_direction(pin2, GPIO_MODE_OUTPUT);
    gpio_set_level(pin2, 0);

    // PWM SETTINGS
    // Create operator
    mcpwm_operator_config_t oper_config = {
        .group_id = timer_config.group_id
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &new_motor.operator));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(new_motor.operator, wheel_motor_timer));

    // Create comparator
    mcpwm_comparator_config_t comp_config = { .flags.update_cmp_on_tez = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(new_motor.operator, &comp_config, &new_motor.comparator));

    // Create generator
    mcpwm_generator_config_t gen1_config = {
        .gen_gpio_num = pwm_pin
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(new_motor.operator, &gen1_config, &new_motor.generator));

    // Set actions
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(new_motor.generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(new_motor.generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, new_motor.comparator, MCPWM_GEN_ACTION_LOW)));

    new_motor.max_pwm_value = timer_config.period_ticks;

    return new_motor;
}

// Private
pcnt_unit_handle_t init_quadure_encoder(gpio_num_t enc_a, gpio_num_t enc_b){
   pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32768,
    };

    // Enable pullups, encoders should be active low
    //gpio_pullup_en(enc_a);
    //gpio_pullup_en(enc_b);

    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    // Chan A
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = enc_a,
        .level_gpio_num = enc_b,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Chan B, 4x sampleing
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = enc_b,
        .level_gpio_num = enc_a,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Enable filtering, this might need to be tuned
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 10000, // Pulses shorter then 1 microsecond are rejected
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    // Start the timer  
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));

    ESP_LOGI("Controller", "Encoder created");

    return pcnt_unit;
}



// Public
void wheel_init_hardware(){
    if(control_initialized){
        ESP_LOGE("Control", "Called control_init_hardware while already initialized");
        return;
    }

    // Create our main timer for control of the wheels
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &wheel_motor_timer));

    // Create the encoded wheels
    wheels[LEFT_WHEEL].motor = init_wheel_motor(LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, LEFT_MOTOR_PWM_PIN);
    wheels[LEFT_WHEEL].encoder = init_quadure_encoder(LEFT_MOTOR_ENCODER_A_PIN, LEFT_MOTOR_ENCODER_B_PIN);
    wheels[RIGHT_WHEEL].motor = init_wheel_motor(RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2, RIGHT_MOTOR_PWM_PIN);
    wheels[RIGHT_WHEEL].encoder = init_quadure_encoder(RIGHT_MOTOR_ENCODER_A_PIN, RIGHT_MOTOR_ENCODER_B_PIN);

    // Enable the timer
    mcpwm_timer_enable(wheel_motor_timer);
    mcpwm_timer_start_stop(wheel_motor_timer, MCPWM_TIMER_START_NO_STOP);

    // Enable the motor encoders
    ESP_ERROR_CHECK(pcnt_unit_start(wheels[LEFT_WHEEL].encoder));
    ESP_ERROR_CHECK(pcnt_unit_start(wheels[RIGHT_WHEEL].encoder));

    control_initialized = 1;
}

// Public
int wheel_get_encoder_pulses(wheel_t wheel, bool reset){
    int pulses = 0;
    pcnt_unit_get_count(wheels[wheel].encoder, &pulses);
    if(reset) pcnt_unit_clear_count(wheels[wheel].encoder);

    return pulses;
}

void wheel_reset_encoder_count(wheel_t wheel){
    pcnt_unit_clear_count(wheels[wheel].encoder);
}


// Public
void wheel_set_speed(wheel_t wheel, int32_t speed){
    motor_t* motor = &wheels[wheel].motor;

    // Change PWM signal
    uint32_t abs_speed = abs(speed);
    if (abs_speed > motor->max_pwm_value) abs_speed = motor->max_pwm_value;
    mcpwm_comparator_set_compare_value(motor->comparator, abs_speed);

    // Change H-bridge
    if(speed > 0){
        gpio_set_level(motor->pin_1, 0);
        gpio_set_level(motor->pin_2, 1);
    }else if(speed < 0){
        gpio_set_level(motor->pin_2, 0);
        gpio_set_level(motor->pin_1, 1);
    }else{
        gpio_set_level(motor->pin_1, 0);
        gpio_set_level(motor->pin_2, 0);
    }
}