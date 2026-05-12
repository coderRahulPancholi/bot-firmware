#include "line_sensors.h"
#include "esp_log.h"
#include "system_monitor.h" 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SENSORS";

LineSensors::LineSensors() {
    for(int i=0; i<NUM_SENSORS; i++) {
        sensor_values[i] = 0;
    }
}

esp_err_t LineSensors::configureADC(i2c_dev_t* adc, uint8_t address, i2c_port_t port, gpio_num_t sda, gpio_num_t scl) {
    esp_err_t err = ads111x_init_desc(
        adc,
        address,
        port,
        static_cast<gpio_num_t>(sda),
        static_cast<gpio_num_t>(scl)
    );
    if (err != ESP_OK) return err;
    
    // Set to maximum speed (860 Samples Per Second) for fast robot reaction times
    err = ads111x_set_data_rate(adc, ADS111X_DATA_RATE_860);
    if (err != ESP_OK) return err;

    // Gain of 4.096V. This is perfect since TCRT5000 output is typically 0 to 3.3V
    err = ads111x_set_gain(adc, ADS111X_GAIN_4V096);
    if (err != ESP_OK) return err;

    // Single-shot mode (we will manually trigger conversions)
    err = ads111x_set_mode(adc, ADS111X_MODE_SINGLE_SHOT);
    if (err != ESP_OK) return err;
    
    return ESP_OK;
}

esp_err_t LineSensors::init(i2c_port_t port, gpio_num_t sda_pin, gpio_num_t scl_pin, uint8_t addr1, uint8_t addr2) {
    ESP_LOGI(TAG, "Initializing Line Sensors (ADS1115)...");

    esp_err_t err = configureADC(&adc1, addr1, port, sda_pin, scl_pin);
    if (err != ESP_OK) {
        SystemMonitor::reportError("SENSORS", "ADC1 (0x48) Init Fail", err);
        return err;
    }

    err = configureADC(&adc2, addr2, port, sda_pin, scl_pin);
    if (err != ESP_OK) {
        SystemMonitor::reportError("SENSORS", "ADC2 (0x49) Init Fail", err);
        return err;
    }

    ESP_LOGI(TAG, "Line Sensors Ready.");
    return ESP_OK;
}

esp_err_t LineSensors::readAll() {
    esp_err_t err;
    int16_t raw_val;

    // 1. Read ADC 1 (Sensors 0 to 3)
    for (uint8_t i = 0; i < 4; i++) {
        // Switch multiplexer to the correct channel (A0, A1, A2, A3 to GND)
        ads111x_set_input_mux(&adc1, (ads111x_mux_t)(ADS111X_MUX_0_GND + i));
        ads111x_start_conversion(&adc1);
        
        // At 860 SPS, conversion takes ~1.2ms. We wait 2ms to be safe.
        vTaskDelay(pdMS_TO_TICKS(2)); 
        
        err = ads111x_get_value(&adc1, &raw_val);
        if (err != ESP_OK) {
            SystemMonitor::reportError("SENSORS", "ADC1 Read Error", err);
            return err;
        }
        
        // Prevent negative values from ADC noise
        sensor_values[i] = (raw_val > 0) ? (uint16_t)raw_val : 0; 
    }

    // 2. Read ADC 2 (Sensors 4 to 5 on channels A0 and A1)
    for (uint8_t i = 0; i < 2; i++) {
        ads111x_set_input_mux(&adc2, (ads111x_mux_t)(ADS111X_MUX_0_GND + i));
        ads111x_start_conversion(&adc2);
        
        vTaskDelay(pdMS_TO_TICKS(2));
        
        err = ads111x_get_value(&adc2, &raw_val);
        if (err != ESP_OK) {
            SystemMonitor::reportError("SENSORS", "ADC2 Read Error", err);
            return err;
        }
        
        sensor_values[i + 4] = (raw_val > 0) ? (uint16_t)raw_val : 0;
    }

    return ESP_OK;
}

uint16_t LineSensors::getValue(uint8_t index) {
    if (index < NUM_SENSORS) {
        return sensor_values[index];
    }
    return 0;
}

const uint16_t* LineSensors::getAllValues() {
    return sensor_values;
}
