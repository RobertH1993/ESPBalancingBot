#include "pid.h"
#include <math.h>

float pid_compute(pid_controller_t *pid, float measurement, float dt, float external_d){
    float error = pid->setpoint - measurement;

    pid->P = pid->Kp * error;
    pid->integral += (error * dt);

    pid->I = pid->Ki * pid->integral;

    if(external_d != 0.0f){
        pid->D = pid->Kd * external_d;
    }else{
        pid->D = pid->Kd * (error - pid->prev_error) / dt;
        pid->prev_error = error;
    }
    
    pid->output = pid->P + pid->I + pid->D;

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