#ifndef _STATE_H
#define _STATE_H

#include <stdint.h>
#include "pid.h"

#define PID_BALANCE 0
#define PID_POSITION 1
#define PID_SPEED 2
#define PID_WHEEL_TRIM 3

typedef enum {
    HOLD_POSITION = 0,
    DRIVE = 1
} robot_status_t;


// State of the full robot
typedef struct{
    robot_status_t status;
    float distance_left; // Distance the left wheel has driven
    float distance_right; // Distance the right wheel has driven
    pid_controller_t pids[4];
} robot_state_t;

extern robot_state_t rstate;

#endif