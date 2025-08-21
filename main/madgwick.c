#include <math.h>
#include "madgwick.h"

static float beta = MADGWICK_BETA;

quaternion_t quaternion = {1.f, 0.f, 0.f, 0.f};

static inline float inv_sqrtf(float x)
{
    return 1.0f / sqrtf(x);
}

void madgwick_update_imu(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float dt)
{
    float q0 = quaternion.w, q1 = quaternion.x, q2 = quaternion.y, q3 = quaternion.z;

    float norm = ax * ax + ay * ay + az * az;
    if (norm > 0.0f)
    {
        norm = inv_sqrtf(norm);
        ax *= norm;
        ay *= norm;
        az *= norm;
    }
    else
    {
        float half_dt = 0.5f * dt;
        float qDot0 = (-q1 * gx - q2 * gy - q3 * gz);
        float qDot1 = (q0 * gx + q2 * gz - q3 * gy);
        float qDot2 = (q0 * gy - q1 * gz + q3 * gx);
        float qDot3 = (q0 * gz + q1 * gy - q2 * gx);
        q0 += qDot0 * half_dt;
        q1 += qDot1 * half_dt;
        q2 += qDot2 * half_dt;
        q3 += qDot3 * half_dt;
        norm = inv_sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        quaternion.w = q0 * norm;
        quaternion.x = q1 * norm;
        quaternion.y = q2 * norm;
        quaternion.z = q3 * norm;
        return;
    }

    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    norm = inv_sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;

    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
    float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
    float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
    float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    norm = inv_sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    quaternion.w = q0 * norm;
    quaternion.x = q1 * norm;
    quaternion.y = q2 * norm;
    quaternion.z = q3 * norm;
}

void quat_to_euler_deg(const quaternion_t *q, float *roll, float *pitch, float *yaw)
{
    float qw = q->w, qx = q->x, qy = q->y, qz = q->z;

    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    *roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / (float) M_PI;

    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f) *pitch = copysignf(90.0f, sinp);
    else *pitch = asinf(sinp) * 180.0f / (float) M_PI;

    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    *yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / (float) M_PI;
}
