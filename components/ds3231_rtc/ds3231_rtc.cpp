#include "ds3231_rtc.h"
#include "system_monitor.h"
#include "esp_log.h"

static const char* TAG = "DS3231_CLASS";

DS3231RTC::DS3231RTC() {
    initialized = false;
}

esp_err_t DS3231RTC::init(i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin) {
    esp_err_t err = ds3231_init_desc(&rtc_dev, port, sda_pin, scl_pin);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DS3231 descriptor on I2C bus!");
        // Report the fault to the LCD screen
        SystemMonitor::reportError("RTC_MODULE", "I2C Init Failed!", err);
        return err;
    }
    
    initialized = true;
    ESP_LOGI(TAG, "DS3231 RTC successfully initialized.");
    return ESP_OK;
}

esp_err_t DS3231RTC::getTime(struct tm* timeinfo) {
    if (!initialized) return ESP_ERR_INVALID_STATE;
    
    esp_err_t err = ds3231_get_time(&rtc_dev, timeinfo);
    if (err != ESP_OK) {
        // If the sensor disconnects mid-race, report it!
        SystemMonitor::reportError("RTC_MODULE", "I2C Read Timeout!", err);
    }
    return err;
}

esp_err_t DS3231RTC::setTime(struct tm* timeinfo) {
    if (!initialized) return ESP_ERR_INVALID_STATE;
    
    esp_err_t err = ds3231_set_time(&rtc_dev, timeinfo);
    if (err != ESP_OK) {
        SystemMonitor::reportError("RTC_MODULE", "Failed to Set Time!", err);
    }
    return err;
}

esp_err_t DS3231RTC::getTemperature(float* temp) {
    if (!initialized) return ESP_ERR_INVALID_STATE;
    
    esp_err_t err = ds3231_get_temp_float(&rtc_dev, temp);
    if (err != ESP_OK) {
        SystemMonitor::reportError("RTC_MODULE", "Temp Sensor Offline!", err);
    }
    return err;
}