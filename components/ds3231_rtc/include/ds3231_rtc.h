#pragma once

#include <time.h>
#include "esp_err.h"
#include "hal/gpio_types.h"
#include "i2cdev.h"
#include "ds3231.h" // The underlying esp-idf-lib driver

class DS3231RTC {
private:
    i2c_dev_t rtc_dev;
    bool initialized;

public:
    DS3231RTC();

    // Initializes the I2C descriptor for the DS3231 (Default Address: 0x68)
    esp_err_t init(i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin);

    // Reads the current time from the RTC chip
    esp_err_t getTime(struct tm* timeinfo);

    // Sets a new time to the RTC chip
    esp_err_t setTime(struct tm* timeinfo);

    // Reads the internal crystal temperature in Celsius
    esp_err_t getTemperature(float* temp);
};