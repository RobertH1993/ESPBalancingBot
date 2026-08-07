#ifndef _STATE_H
#define _STATE_H

#include <stdint.h>
#include "pid.h"

#define PID_BALANCE 0
#define PID_SPEED 1
#define PID_WHEEL_TRIM 2



// State of the full robot
typedef struct{
    float distance_left; // Distance the left wheel has driven
    float distance_right; // Distance the right wheel has driven
    pid_controller_t pids[3];
} robot_state_t;

extern robot_state_t rstate;

#endif