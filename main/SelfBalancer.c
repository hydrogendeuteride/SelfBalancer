#include <math.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mpu6050.h"
#include "madgwick.h"
#include "motor.h"
#include "balance_controller.h"
#include "control_config.h"

static const char *TAG = "SelfBalancer";

static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

void mpu6050_task(void *arg)
{
    mpu6050_data_t sensor_data;
    float pitch_rad_exact;

    uint64_t last_t_us = esp_timer_get_time();

    static int num = 0;

    balance_pid_t pid;
    balance_pid_init(&pid, ANGLE_KP, ANGLE_KI, ANGLE_KD);
    bool armed = false;

    while (1)
    {
        if (mpu6050_read_data(&sensor_data) == ESP_OK)
        {
            // dt = s
            uint64_t now_us = esp_timer_get_time();
            float dt = (now_us - last_t_us) / 1000000.0f;
            last_t_us = now_us;
            if (dt <= 0.0f || dt > 0.2f) dt = 0.01f;

            const float DEG2RAD = (float) M_PI / 180.0f;
            madgwick_update_imu(sensor_data.gyro_x * DEG2RAD,
                                sensor_data.gyro_y * DEG2RAD,
                                sensor_data.gyro_z * DEG2RAD,
                                sensor_data.accel_x,
                                sensor_data.accel_y,
                                sensor_data.accel_z,
                                dt);

            pitch_rad_exact = quat_pitch_rad(&quaternion);
            const float RAD2DEG = 180.0f / (float) M_PI;
            float pitch_deg = pitch_rad_exact * RAD2DEG;

            float gyro_dps = 0.0f;
            switch (PITCH_GYRO_AXIS)
            {
                case 0: gyro_dps = sensor_data.gyro_x;
                    break;
                case 1: gyro_dps = sensor_data.gyro_y;
                    break;
                case 2: gyro_dps = sensor_data.gyro_z;
                    break;
                default: gyro_dps = sensor_data.gyro_y;
                    break;
            }
            float pitch_rate = PITCH_GYRO_SIGN * gyro_dps * DEG2RAD; // rad/s

            if (!armed)
            {
                if (fabsf(pitch_rad_exact) < ARM_TILT_RAD) // 8 deg
                {
                    armed = true;
                    balance_pid_reset(&pid);
                }
            }
            else if (fabsf(pitch_rad_exact) > DISARM_TILT_RAD) //35 deg
            {
                armed = false;
                balance_pid_reset(&pid);
                motor_brake();
            }

            float theta_sp = 0.0f;
            float qy_signed = (quaternion.w >= 0.0f) ? quaternion.y : -quaternion.y;
            float pitch_err_quat = 2.0f * qy_signed;
            float u = balance_pid_update(&pid, theta_sp, pitch_err_quat, pitch_rate, dt);

            float left = u;
            float right = u;
            motor_set(left, right, armed);

            num++;
            if (num % MAX(1, (100 / CONTROL_LOG_HZ)) == 0)
            {
                ESP_LOGI(TAG, "pitch≈%.3frad (%.1f deg) rate=%.2frad/s u=%.2f armed=%d", pitch_rad_exact, pitch_deg,
                         pitch_rate, u, armed);
                num = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "SelfBalancer start");

    ESP_ERROR_CHECK(motor_init());
    ESP_LOGI(TAG, "Motor init success");

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C init success");

    vTaskDelay(pdMS_TO_TICKS(100));

    if (mpu6050_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU6050 init failed");
        return;
    }


    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 5, NULL);
}
