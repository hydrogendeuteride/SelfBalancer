#pragma once

typedef struct
{
    float kp, ki, kd;
    float integ;
    float prev_u;
} balance_pid_t;

void balance_pid_init(balance_pid_t *pid, float kp, float ki, float kd);

void balance_pid_reset(balance_pid_t *pid);

float balance_pid_update(balance_pid_t *pid,
                         float theta_sp, // desired pitch [rad]
                         float theta, // measured pitch [rad]
                         float theta_dot, // measured pitch rate [rad/s]
                         float dt);

