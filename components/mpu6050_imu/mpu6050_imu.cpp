#include "mpu6050_imu.h"
#include "system_monitor.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "IMU_CLASS";

MPU6050IMU::MPU6050IMU() {
    initialized = false;
    memset(&imu_dev, 0, sizeof(mpu6050_dev_t));
}

esp_err_t MPU6050IMU::init(i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin) {
    // 1. Setup the I2C descriptor
    esp_err_t err = mpu6050_init_desc(&imu_dev, MPU6050_I2C_ADDRESS_LOW, port, sda_pin, scl_pin);
    if (err != ESP_OK) {
        SystemMonitor::reportError("MPU6050", "I2C Descriptor Failed", err);
        return err;
    }

    // 2. Wake up the chip and configure standard settings
    err = mpu6050_init(&imu_dev);
    if (err != ESP_OK) {
        SystemMonitor::reportError("MPU6050", "Chip Init Failed", err);
        return err;
    }

    initialized = true;
    ESP_LOGI(TAG, "MPU6050 IMU successfully initialized.");
    return ESP_OK;
}

esp_err_t MPU6050IMU::getAcceleration(float& ax, float& ay, float& az) {
    if (!initialized) return ESP_ERR_INVALID_STATE;

    mpu6050_acceleration_t accel;
    esp_err_t err = mpu6050_get_acceleration(&imu_dev, &accel);
    
    if (err == ESP_OK) {
        ax = accel.x;
        ay = accel.y;
        az = accel.z;
    } else {
        SystemMonitor::reportError("MPU6050", "Accel Read Failed!", err);
    }
    return err;
}

esp_err_t MPU6050IMU::getRotation(float& rx, float& ry, float& rz) {
    if (!initialized) return ESP_ERR_INVALID_STATE;

    mpu6050_rotation_t rot;
    esp_err_t err = mpu6050_get_rotation(&imu_dev, &rot);
    
    if (err == ESP_OK) {
        rx = rot.x;
        ry = rot.y;
        rz = rot.z;
    } else {
        SystemMonitor::reportError("MPU6050", "Gyro Read Failed!", err);
    }
    return err;
}

esp_err_t MPU6050IMU::getTemperature(float& temp) {
    if (!initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t err = mpu6050_get_temperature(&imu_dev, &temp);
    if (err != ESP_OK) {
        SystemMonitor::reportError("MPU6050", "Temp Read Failed!", err);
    }
    return err;
}