#ifndef _WHEEL_H
#define _WHEEL_H
#include <stdbool.h>
#include <stdint.h>

typedef enum{
    LEFT_WHEEL = 0,
    RIGHT_WHEEL = 1,
} wheel_t;

/**
 * Initialize the hardware for the wheels.
 * This function must be called before any other wheel functions.
 */
void wheel_init_hardware();

/**
 * Set the speed of a wheel to a certain PWM value. 
 * @param wheel The wheel to set the speed of
 * @param pwm_value The PWM value to set the speed to, this can be positive or negative and the sign will determine the direction of the wheel.
 */
void wheel_set_speed(wheel_t wheel, int32_t pwm_value);

/**
 * Get the number of encoder pulses for a wheel
 * @param wheel The wheel to get the encoder pulses of
 * @param reset Whether to reset the encoder count to 0 after reading
 * @return The number of encoder pulses
 */
int wheel_get_encoder_pulses(wheel_t wheel, bool reset);

/**
 * Reset the encoder count for a wheel to 0.
 * @param wheel The wheel to reset the encoder count of
 */
void wheel_reset_encoder_count(wheel_t wheel);

#endif