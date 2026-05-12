#include "battery_monitor.h"
#include "system_monitor.h"
#include "esp_log.h"
#include <algorithm> // for std::clamp

static const char *TAG = "BATTERY";

BatteryMonitor::BatteryMonitor(float r1_kOhms, float r2_kOhms, float v_max, float v_min) {
    // Voltage Divider Formula: V_in = V_out * ((R1 + R2) / R2)
    this->divider_multiplier = (r1_kOhms + r2_kOhms) / r2_kOhms;
    this->max_voltage = v_max;
    this->min_voltage = v_min;
    this->calibrated = false;
}

esp_err_t BatteryMonitor::init(adc_unit_t unit, adc_channel_t channel) {
    this->adc_unit = unit;
    this->adc_channel = channel;
    esp_err_t err;

    // 1. Initialize the ADC Unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = unit,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    err = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (err != ESP_OK) {
        SystemMonitor::reportError("BATTERY", "ADC Unit Init Failed", err);
        return err;
    }

    // 2. Configure the Channel
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12, 
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(adc_handle, channel, &config);
    if (err != ESP_OK) {
        SystemMonitor::reportError("BATTERY", "ADC Channel Failed", err);
        return err;
    }

    // 3. Setup Hardware Calibration
    this->calibrated = initCalibration(unit, channel, ADC_ATTEN_DB_12);

    ESP_LOGI(TAG, "Battery Monitor Initialized.");
    return ESP_OK;
}

bool BatteryMonitor::initCalibration(adc_unit_t unit, adc_channel_t chan, adc_atten_t atten) {
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .default_vref = 1100,
    };
    
    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);
    if (ret == ESP_OK) {
        return true;
    }
    
    ESP_LOGW(TAG, "Hardware calibration not supported on this board. Using raw values.");
    return false;
}

float BatteryMonitor::getVoltage() {
    int raw_val;
    // 1. Hardware Check: Did the ADC read successfully?
    esp_err_t err = adc_oneshot_read(adc_handle, adc_channel, &raw_val);
    if (err != ESP_OK) {
        SystemMonitor::reportError("BATTERY", "ADC Read Failed", err);
        return 0.0f; // Return 0V so the robot knows to safely stop
    }

    int voltage_mv = 0;
    
    if (calibrated) {
        // 2. Hardware Check: Did the calibration math succeed?
        err = adc_cali_raw_to_voltage(cali_handle, raw_val, &voltage_mv);
        if (err != ESP_OK) {
            SystemMonitor::reportError("BATTERY", "Cali Conv Failed", err);
            // Fallback to raw estimation so the robot doesn't instantly die
            voltage_mv = (raw_val * 3100) / 4095;
        }
    } else {
        voltage_mv = (raw_val * 3100) / 4095; 
    }

    float pin_voltage = (float)voltage_mv / 1000.0f;
    return pin_voltage * divider_multiplier;
}

int BatteryMonitor::getPercentage() {
    float current_v = getVoltage();
    
    // Calculate percentage
    float percentage = ((current_v - min_voltage) / (max_voltage - min_voltage)) * 100.0f;
    
    // Clamp between 0 and 100 so you don't get 105% or -5%
    if (percentage > 100.0f) percentage = 100.0f;
    if (percentage < 0.0f) percentage = 0.0f;
    
    return (int)percentage;
}