#pragma once

#include "esp_err.h"
#include "hal/gpio_types.h"
#include "i2cdev.h"
#include "mpu6050.h" 

class MPU6050IMU {
private:
    mpu6050_dev_t imu_dev;
    bool initialized;

public:
    MPU6050IMU();

    // Initializes the IMU descriptor and wakes up the chip
    esp_err_t init(i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin);

    // Reads Acceleration in G's (1.0 = standard earth gravity)
    esp_err_t getAcceleration(float& ax, float& ay, float& az);

    // Reads Gyroscope Rotation in degrees per second
    esp_err_t getRotation(float& rx, float& ry, float& rz);

    // Reads the internal silicon temperature (Celsius)
    esp_err_t getTemperature(float& temp);
};