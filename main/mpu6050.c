#include <math.h>
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050.h"

static const char *TAG = "MPU6050";

static float gyro_scale_factor = 131.0f;
static float accel_scale_factor = 16384.0f;

static esp_err_t mpu6050_read_byte(uint8_t reg_addr, uint8_t *data)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR,
                                        &reg_addr, 1, data, 1,
                                        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR,
                                      write_buf, sizeof(write_buf),
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR,
                                        &reg_addr, 1, data, len,
                                        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_init(void)
{
    esp_err_t ret;
    uint8_t who_am_i;

    ret = mpu6050_read_byte(MPU6050_WHO_AM_I, &who_am_i);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read who am_i");
        return ret;
    }

    if (who_am_i != 0x68)
    {
        ESP_LOGE(TAG, "Failed to find MPU6050. WHO_AM_I: 0x%02X", who_am_i);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MPU6050 (WHO_AM_I: 0x%02X)", who_am_i);

    ret = mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Power manage failed");
        return ret;
    }

    ret = mpu6050_write_byte(MPU6050_GYRO_CONFIG, GYRO_SCALE_250DPS << 3);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Gyro config failed");
        return ret;
    }

    ret = mpu6050_write_byte(MPU6050_ACCEL_CONFIG, ACCEL_SCALE_2G << 3);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Accel config failed");
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 initialized");
    return ESP_OK;
}

esp_err_t mpu6050_set_gyro_scale(gyro_scale_t scale)
{
    esp_err_t ret = mpu6050_write_byte(MPU6050_GYRO_CONFIG, scale << 3);

    if (ret == ESP_OK)
    {
        switch (scale)
        {
            case GYRO_SCALE_250DPS: gyro_scale_factor = 131.0f;
                break;
            case GYRO_SCALE_500DPS: gyro_scale_factor = 65.5f;
                break;
            case GYRO_SCALE_1000DPS: gyro_scale_factor = 32.8f;
                break;
            case GYRO_SCALE_2000DPS: gyro_scale_factor = 16.4f;
                break;
        }
        ESP_LOGI(TAG, "Gyro scale set: %d dps full-scale", 250 << scale);
    }
    return ret;
}

esp_err_t mpu6050_set_accel_scale(accel_scale_t scale)
{
    esp_err_t ret = mpu6050_write_byte(MPU6050_ACCEL_CONFIG, scale << 3);

    if (ret == ESP_OK)
    {
        switch (scale)
        {
            case ACCEL_SCALE_2G: accel_scale_factor = 16384.0f;
                break;
            case ACCEL_SCALE_4G: accel_scale_factor = 8192.0f;
                break;
            case ACCEL_SCALE_8G: accel_scale_factor = 4096.0f;
                break;
            case ACCEL_SCALE_16G: accel_scale_factor = 2048.0f;
                break;
        }
        ESP_LOGI(TAG, "Accel scale set: %d g full-scale", 2 << scale);
    }

    return ret;
}

esp_err_t mpu6050_read_data(mpu6050_data_t *data)
{
    uint8_t raw_data[14];
    esp_err_t ret = mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, raw_data, 14);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read accelerometer data");
        return ret;
    }

    int16_t accel_x_raw = (raw_data[0] << 8) | raw_data[1];
    int16_t accel_y_raw = (raw_data[2] << 8) | raw_data[3];
    int16_t accel_z_raw = (raw_data[4] << 8) | raw_data[5];
    int16_t temp_raw = (raw_data[6] << 8) | raw_data[7];
    int16_t gyro_x_raw = (raw_data[8] << 8) | raw_data[9];
    int16_t gyro_y_raw = (raw_data[10] << 8) | raw_data[11];
    int16_t gyro_z_raw = (raw_data[12] << 8) | raw_data[13];

    data->accel_x = accel_x_raw / accel_scale_factor;
    data->accel_y = accel_y_raw / accel_scale_factor;
    data->accel_z = accel_z_raw / accel_scale_factor;

    data->gyro_x = gyro_x_raw / gyro_scale_factor;
    data->gyro_y = gyro_y_raw / gyro_scale_factor;
    data->gyro_z = gyro_z_raw / gyro_scale_factor;

    data->temperature = temp_raw / 340.0f + 36.53f;

    return ESP_OK;
}
