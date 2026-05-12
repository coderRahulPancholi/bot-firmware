#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "i2cdev.h"
#include "ads111x.h"

#define NUM_SENSORS 6

class LineSensors {
private:
    i2c_dev_t adc1; // Handles sensors 0, 1, 2, 3
    i2c_dev_t adc2; // Handles sensors 4, 5
    
    uint16_t sensor_values[NUM_SENSORS];

    // Helper method to configure each ADS1115 chip
    esp_err_t configureADC(i2c_dev_t* adc, uint8_t address, i2c_port_t port, gpio_num_t sda, gpio_num_t scl);

public:
    LineSensors();

    // Initializes both ADCs on the I2C bus
    // Default addresses match ADDR->GND and ADDR->VCC
    esp_err_t init(i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin, 
                   uint8_t addr1 = ADS111X_ADDR_GND, 
                   uint8_t addr2 = ADS111X_ADDR_VCC);

    // Reads all 6 analog channels sequentially and updates the internal array
    esp_err_t readAll();

    // Returns the last read value of a specific sensor (0 to 5)
    uint16_t getValue(uint8_t index);

    // Returns a pointer to the entire array (useful for PID math)
    const uint16_t* getAllValues();
};