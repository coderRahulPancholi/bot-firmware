#pragma once

#include "esp_err.h"
#include "hal/gpio_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

class BatteryMonitor {
private:
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle;
    bool calibrated;
    
    adc_unit_t adc_unit;
    adc_channel_t adc_channel;
    
    float divider_multiplier;
    float max_voltage;
    float min_voltage;

    bool initCalibration(adc_unit_t unit, adc_channel_t chan, adc_atten_t atten);

public:
    // r1: Resistor connected to Battery (+)
    // r2: Resistor connected to GND (-)
    // v_max: Voltage at 100% (e.g., 8.4V for 2S LiPo)
    // v_min: Voltage at 0% (e.g., 6.4V for 2S LiPo)
    BatteryMonitor(float r1_kOhms, float r2_kOhms, float v_max, float v_min);
    
    // Initializes the ADC hardware
    esp_err_t init(adc_unit_t unit, adc_channel_t channel);
    
    // Returns the actual calculated battery voltage
    float getVoltage();
    
    // Returns 0-100% based on your max/min limits
    int getPercentage();
};