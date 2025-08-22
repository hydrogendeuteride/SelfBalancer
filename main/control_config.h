#pragma once

#include <math.h>
#include "driver/gpio.h"

#define CONTROL_LOG_HZ           50      // telemetry/log rate

#define ANGLE_KP                 25.0f
#define ANGLE_KI                 0.0f
#define ANGLE_KD                 0.7f

#define U_MAX                    0.75f   //max motor power set [0, 1]
#define U_SLEW                   2.5f    //max motor power deriv set

#define ARM_TILT_RAD             (8.0f * (float)M_PI / 180.0f)   // 8 deg
#define DISARM_TILT_RAD          (35.0f * (float)M_PI / 180.0f)  // 35 deg

// 0: gx, 1: gy, 2: gz; SIGN: +1 / -1
#define PITCH_GYRO_AXIS          1
#define PITCH_GYRO_SIGN          (+1.0f)

//-------------------------------------------------------------------------------

#define DIREADTIME__DMS 3

#define AIN1_PIN   25
#define AIN2_PIN   26
#define BIN1_PIN   27
#define BIN2_PIN   14

#define LEFT_MOTOR_INVERTED      0
#define RIGHT_MOTOR_INVERTED     1

#define LEDC_TIMER          LEDC_TIMER_0
#if ESP_IDF_VERSION_MAJOR >= 5
  #define LEDC_MODE         LEDC_LOW_SPEED_MODE
#else
  #define LEDC_MODE         LEDC_HIGH_SPEED_MODE
#endif
#define LEDC_DUTY_RES       LEDC_TIMER_10_BIT   // 0~1023
#define LEDC_FREQUENCY_HZ   20000               // 20 kHz

#define CH_AIN1             LEDC_CHANNEL_0
#define CH_AIN2             LEDC_CHANNEL_1
#define CH_BIN1             LEDC_CHANNEL_2
#define CH_BIN2             LEDC_CHANNEL_3
