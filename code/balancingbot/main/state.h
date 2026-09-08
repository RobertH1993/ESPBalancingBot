#ifndef _STATE_H
#define _STATE_H

#include <stdint.h>
#include "pid.h"

#define PID_BALANCE 0
#define PID_SPEED 1
#define PID_WHEEL_TRIM 2


// High level operating mode of the robot, driven by the control task state machine
typedef enum{
    ROBOT_STATE_INIT = 0,   // Hardware / IMU is being brought up
    ROBOT_STATE_CALIBRATING,// IMU calibration in progress, hold steady
    ROBOT_STATE_SETTLING,   // Waiting until the robot is held upright and steady
    ROBOT_STATE_BALANCING,  // Actively balancing
    ROBOT_STATE_FALLEN,     // Tipped over, motors off, waiting to be set upright again
} robot_mode_t;

// State of the full robot
typedef struct{
    robot_mode_t mode; // Current operating mode (see robot_mode_t)
    float distance_left; // Distance the left wheel has driven
    float distance_right; // Distance the right wheel has driven
    float target_speed; // Target forward/backward speed in cm/s (controlled by user)
    float target_turn_rate; // Target turn rate in degrees/s (controlled by user)
    pid_controller_t pids[3];
} robot_state_t;

extern robot_state_t rstate;

#endif