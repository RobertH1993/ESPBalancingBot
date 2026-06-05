#ifndef _PID_H
#define _PID_H
#include <stdint.h>


typedef struct{
    // Gains
    volatile float Kp;
    volatile float Ki;
    volatile float Kd;

    // Last calculated PID values
    float P;
    float I;
    float D;
    float output;

    uint8_t no_d_counter;

    // Max value that the PID controller may output, this is used for anti I windup, if we output the max PWM signal we dont want I to windup further
    float max_output;

    float setpoint;
    float integral;
    float prev_error;
} pid_controller_t;

// Calculate a new PID value
float pid_compute(pid_controller_t *pid, float measurement, float dt);

void pid_change_setpoint(pid_controller_t *pid, float setpoint);

#endif