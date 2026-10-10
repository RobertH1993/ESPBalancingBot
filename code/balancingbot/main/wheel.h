#ifndef _WHEEL_H
#define _WHEEL_H
#include <stdint.h>

typedef enum{
    LEFT_WHEEL = 0,
    RIGHT_WHEEL = 1,
} wheel_t;




void wheel_init_hardware();

void wheel_set_speed(wheel_t wheel, int32_t speed);
int wheel_get_encoder_pulses(wheel_t wheel, bool reset);
void wheel_reset_encoder_count(wheel_t wheel);

#endif