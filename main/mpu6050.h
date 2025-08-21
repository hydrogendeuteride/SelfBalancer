#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_ADDR                0x68
#define MPU6050_WHO_AM_I            0x75
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_TEMP_OUT_H          0x41
#define MPU6050_GYRO_XOUT_H         0x43

typedef enum
{
    GYRO_SCALE_250DPS = 0,
    GYRO_SCALE_500DPS = 1,
    GYRO_SCALE_1000DPS = 2,
    GYRO_SCALE_2000DPS = 3
} gyro_scale_t;

typedef enum
{
    ACCEL_SCALE_2G = 0,
    ACCEL_SCALE_4G = 1,
    ACCEL_SCALE_8G = 2,
    ACCEL_SCALE_16G = 3
} accel_scale_t;

typedef struct
{
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z; // deg/s
    float temperature;
} mpu6050_data_t;

esp_err_t mpu6050_init(void);

esp_err_t mpu6050_set_gyro_scale(gyro_scale_t scale);

esp_err_t mpu6050_set_accel_scale(accel_scale_t scale);

esp_err_t mpu6050_read_data(mpu6050_data_t *data);
