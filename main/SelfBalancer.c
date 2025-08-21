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

static const char *TAG = "MPU6050";

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

static void calculate_tilt_angles(mpu6050_data_t *data, float *roll, float *pitch)
{
    *roll = atan2f(data->accel_y, data->accel_z) * 180.0f / (float) M_PI;
    *pitch = atan2f(-data->accel_x, sqrtf(data->accel_y * data->accel_y + data->accel_z * data->accel_z)) * 180.0f / (
                 float) M_PI;
}

void mpu6050_task(void *arg)
{
    mpu6050_data_t sensor_data;
    float roll_acc, pitch_acc;
    float roll, pitch, yaw;

    uint64_t last_t_us = esp_timer_get_time();

    static int num = 0;

    while (1)
    {
        if (mpu6050_read_data(&sensor_data) == ESP_OK)
        {
            // dt = s
            uint64_t now_us = esp_timer_get_time();
            float dt = (now_us - last_t_us) / 1000000.0f;
            last_t_us = now_us;
            if (dt <= 0.0f || dt > 0.2f) dt = 0.01f;

            calculate_tilt_angles(&sensor_data, &roll_acc, &pitch_acc);

            const float DEG2RAD = (float) M_PI / 180.0f;
            madgwick_update_imu(sensor_data.gyro_x * DEG2RAD,
                                sensor_data.gyro_y * DEG2RAD,
                                sensor_data.gyro_z * DEG2RAD,
                                sensor_data.accel_x,
                                sensor_data.accel_y,
                                sensor_data.accel_z,
                                dt);

            quat_to_euler_deg(&quaternion, &roll, &pitch, &yaw);

            num++;
            if (num % 100 == 0)
            {
                ESP_LOGI(TAG, "=== MPU6050 Sensor data ===");
                ESP_LOGI(TAG, "Accel (g): X=%.3f Y=%.3f Z=%.3f", sensor_data.accel_x, sensor_data.accel_y,
                         sensor_data.accel_z);
                ESP_LOGI(TAG, "Gyro  (dps): X=%.2f Y=%.2f Z=%.2f", sensor_data.gyro_x, sensor_data.gyro_y,
                         sensor_data.gyro_z);
                ESP_LOGI(TAG, "Temp: %.2f C", sensor_data.temperature);
                ESP_LOGI(TAG, "Accel tilt: Roll=%.1f Pitch=%.1f", roll_acc, pitch_acc);
                ESP_LOGI(TAG, "Madgwick:  Roll=%.1f Pitch=%.1f Yaw=%.1f", roll, pitch, yaw);
                num = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "MPU6050 자이로스코프 + Madgwick 테스트 시작");

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C 초기화 완료");

    vTaskDelay(pdMS_TO_TICKS(100));

    if (mpu6050_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU6050 초기화 실패");
        return;
    }

    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "MPU6050 태스크 시작됨");
}
