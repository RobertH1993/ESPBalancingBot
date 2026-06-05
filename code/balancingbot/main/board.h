#ifndef _BOARD_H
#define _BOARD_H

#define I2C_MASTER_0_SDA_IO 32
#define I2C_MASTER_0_SCL_IO 33


#define QMI8658_I2C_ADDRESS_L 0x6a
#define QMI8658_I2C_ADDRESS_H 0x6b

// Left motor settings
#define LEFT_MOTOR_PWM_PIN 26           // PWM Speed control pin
#define LEFT_MOTOR_PIN_1 22             // H-Bridge control pin
#define LEFT_MOTOR_PIN_2 23             // H-Bridge control pin
#define LEFT_MOTOR_ENCODER_A_PIN 27     // Encoder pin
#define LEFT_MOTOR_ENCODER_B_PIN 16     // Encoder pin

// Right motor settings
#define RIGHT_MOTOR_PWM_PIN 25          // PWM Speed control pin
#define RIGHT_MOTOR_PIN_1 21            // H-Bridge control pin
#define RIGHT_MOTOR_PIN_2 17            // H-Bridge control pin
#define RIGHT_MOTOR_ENCODER_A_PIN 35    // Encoder pin
#define RIGHT_MOTOR_ENCODER_B_PIN 34    // Encoder pin

#endif