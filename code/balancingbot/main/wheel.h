#ifndef _WHEEL_H
#define _WHEEL_H
#include <stdint.h>
#include "driver/pulse_cnt.h" //DEBUG
#include "driver/gpio.h" // DEBUG

typedef enum{
    LEFT_WHEEL = 0,
    RIGHT_WHEEL = 1,
    BOTH = 99,
} wheel_t;


pcnt_unit_handle_t init_quadure_encoder(gpio_num_t enc_a, gpio_num_t enc_b); //DEBUG

void wheel_init_hardware();

void wheel_set_speed(wheel_t wheel, int32_t speed);
int wheel_get_encoder_pulses(wheel_t wheel, bool reset);
void wheel_reset_encoder_count(wheel_t wheel);

#endif