#include <math.h>
#include <stdlib.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "motor.h"
#include "control_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "DRV8833"

typedef enum
{
    MOTOR_COAST = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_BRAKE
} motor_dir_t;

typedef struct
{
    gpio_num_t in1, in2;
    ledc_channel_t ch1, ch2;
    motor_dir_t last_dir;
} motor_t;

static motor_t s_left;
static motor_t s_right;

static void drv_timer_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    (void)ledc_timer_config(&t);
}

static void drv_channel_init_one(gpio_num_t pin, ledc_channel_t ch)
{
    ledc_channel_config_t c = {
        .gpio_num = pin,
        .speed_mode = LEDC_MODE,
        .channel = ch,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    (void)ledc_channel_config(&c);
}

static inline void set_duty(ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(LEDC_MODE, ch, duty);
    ledc_update_duty(LEDC_MODE, ch);
}

static inline uint32_t duty_from_percent(int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    const uint32_t duty_max = (1 << LEDC_DUTY_RES) - 1; // 1023
    return (percent * duty_max) / 100;
}

static void motor_apply(const motor_t *m, motor_dir_t dir, int speed_percent)
{
    uint32_t duty = duty_from_percent(speed_percent);

    if ((m->last_dir == MOTOR_FORWARD && dir == MOTOR_BACKWARD) ||
        (m->last_dir == MOTOR_BACKWARD && dir == MOTOR_FORWARD))
    {
        set_duty(m->ch1, 0);
        set_duty(m->ch2, 0);
        vTaskDelay(pdMS_TO_TICKS(DIREADTIME__DMS));
    }

    switch (dir)
    {
        case MOTOR_FORWARD:
            set_duty(m->ch1, duty);
            set_duty(m->ch2, 0);
            break;
        case MOTOR_BACKWARD:
            set_duty(m->ch1, 0);
            set_duty(m->ch2, duty);
            break;
        case MOTOR_BRAKE:
            set_duty(m->ch1, duty_from_percent(100));
            set_duty(m->ch2, duty_from_percent(100));
            break;
        case MOTOR_COAST:
        default:
            set_duty(m->ch1, 0);
            set_duty(m->ch2, 0);
            break;
    }
}

static void motor_run(motor_t *m, motor_dir_t dir, int speed_percent)
{
    motor_apply(m, dir, speed_percent);
    m->last_dir = dir;
}

static inline int pct_from_effort(float effort)
{
    if (effort < 0.0f) effort = -effort;
    if (effort > 1.0f) effort = 1.0f;
    int pct = (int)lrintf(effort * 100.0f);
    if (pct > 100) pct = 100;
    return pct;
}

static inline motor_dir_t dir_from_effort(float effort, bool inverted)
{
    const float eps = 1e-4f;
    if (fabsf(effort) < eps)
    {
        return MOTOR_COAST;
    }
    bool forward = effort > 0.0f;
    if (inverted)
    {
        forward = !forward;
    }
    return forward ? MOTOR_FORWARD : MOTOR_BACKWARD;
}

esp_err_t motor_init(void)
{
    drv_timer_init();

    drv_channel_init_one(AIN1_PIN, CH_AIN1);
    drv_channel_init_one(AIN2_PIN, CH_AIN2);
    drv_channel_init_one(BIN1_PIN, CH_BIN1);
    drv_channel_init_one(BIN2_PIN, CH_BIN2);

    s_left.in1 = AIN1_PIN;
    s_left.in2 = AIN2_PIN;
    s_left.ch1 = CH_AIN1;
    s_left.ch2 = CH_AIN2;
    s_left.last_dir = MOTOR_COAST;

    s_right.in1 = BIN1_PIN;
    s_right.in2 = BIN2_PIN;
    s_right.ch1 = CH_BIN1;
    s_right.ch2 = CH_BIN2;
    s_right.last_dir = MOTOR_COAST;

    motor_apply(&s_left, MOTOR_COAST, 0);
    motor_apply(&s_right, MOTOR_COAST, 0);

    ESP_LOGI(TAG, "Initialized DRV8833 (PWM %d Hz, %d-bit)", (int)LEDC_FREQUENCY_HZ, (int)LEDC_DUTY_RES);
    return ESP_OK;
}

void motor_brake(void)
{
    motor_run(&s_left, MOTOR_BRAKE, 100);
    motor_run(&s_right, MOTOR_BRAKE, 100);
}

void motor_set(float left, float right, bool armed)
{
    if (!armed)
    {
        motor_brake();
        return;
    }

    // Left
    motor_dir_t ldir = dir_from_effort(left, LEFT_MOTOR_INVERTED != 0);
    int lpct = pct_from_effort(left);
    motor_run(&s_left, ldir, lpct);

    // Right
    motor_dir_t rdir = dir_from_effort(right, RIGHT_MOTOR_INVERTED != 0);
    int rpct = pct_from_effort(right);
    motor_run(&s_right, rdir, rpct);
}
