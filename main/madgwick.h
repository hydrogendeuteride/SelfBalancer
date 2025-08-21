#pragma once

#include <stdint.h>

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

#ifndef MADGWICK_BETA
#define MADGWICK_BETA 0.12f
#endif

extern quaternion_t quaternion;

void madgwick_update_imu(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt);

void quat_to_euler_deg(const quaternion_t *q, float *roll, float *pitch, float *yaw);

