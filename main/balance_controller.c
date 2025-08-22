#include <math.h>
#include "balance_controller.h"
#include "control_config.h"

void balance_pid_init(balance_pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integ = 0.0f;
    pid->prev_u = 0.0f;
}

void balance_pid_reset(balance_pid_t *pid)
{
    pid->integ = 0.0f;
    pid->prev_u = 0.0f;
}

float balance_pid_update(balance_pid_t *pid, float theta_sp, float theta, float theta_dot, float dt)
{
    float err = theta_sp - theta;
    pid->integ += err * dt;

    float u = pid->kp * err + pid->kd * (-theta_dot) + pid->ki * pid->integ;

    float u_sat = fmaxf(fminf(u, U_MAX), -U_MAX); //max motor power set [0, 1]
    if (pid->ki > 1e-6f && u != u_sat)
    {
        pid->integ -= (u - u_sat) / pid->ki;
    }

    float du_max = U_SLEW * dt;
    float du = fmaxf(fminf(u_sat - pid->prev_u, du_max), -du_max); //max motor power deriv set
    pid->prev_u += du;
    return pid->prev_u;
}

