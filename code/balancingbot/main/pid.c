#include "pid.h"
#include <math.h>

float pid_compute(pid_controller_t *pid, float measurement, float dt){
    float error = pid->setpoint - measurement;

    pid->P = pid->Kp * error;
    pid->integral += (error * dt);

    pid->I = pid->Ki * pid->integral;

    pid->D = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    if(pid->no_d_counter > 0){
        pid->output = pid->P + pid->I;
        pid->no_d_counter -= 1;
    }else{
        pid->output = pid->P + pid->I + pid->D;
    }

    // Clamp max pid output
    if(pid->output > pid->max_output){
        pid->output = pid->max_output;
        pid->integral -= error * dt; // Anti windup
    }else if(pid->output < -pid->max_output){
        pid->output = -pid->max_output;
        pid->integral -= error * dt; // Anti windup
    }


    return pid->output;
}

void pid_change_setpoint(pid_controller_t *pid, float setpoint){
    if((pid->setpoint > 0 && setpoint < 0) || (pid->setpoint < 0 && setpoint > 0)){
        pid->integral = 0;
    }
    pid->setpoint = setpoint;
}