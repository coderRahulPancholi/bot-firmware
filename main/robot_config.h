#pragma once

#include "driver/gpio.h"
#include "hal/adc_types.h"
#include "driver/i2c.h"

namespace Config {

    // ==========================================
    // 1. I2C BUS MAPPING
    // ==========================================
    constexpr i2c_port_t I2C_PORT    = I2C_NUM_0;
    constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_21;
    constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_22;

    constexpr uint8_t PCF8574_ADDR   = 0x20;
    constexpr uint8_t ADS1115_LEFT   = 0x48;
    constexpr uint8_t ADS1115_RIGHT  = 0x49;
    
    constexpr uint8_t DS3231_ADDR    = 0x68;
    // CRITICAL: Solder the AD0 pad on the MPU6050 to 3.3V to change its 
    // address to 0x69, otherwise it will conflict with the RTC at 0x68!
    constexpr uint8_t MPU6050_ADDR   = 0x69; 

    // ==========================================
    // 2. MOTOR & ENCODER PINS
    // ==========================================
    constexpr gpio_num_t MOTOR_PWM_A = GPIO_NUM_32;
    constexpr gpio_num_t MOTOR_PWM_B = GPIO_NUM_33;

    constexpr gpio_num_t ENC_L_A     = GPIO_NUM_34;
    constexpr gpio_num_t ENC_L_B     = GPIO_NUM_35;
    constexpr gpio_num_t ENC_R_A     = GPIO_NUM_36;
    constexpr gpio_num_t ENC_R_B     = GPIO_NUM_39;

    // ==========================================
    // 3. PHYSICAL ODOMETRY
    // ==========================================
    // Encoder = 360 ticks/rev at the wheel.
    // Formula: ticks_per_rev = BASE_PPR × 4 (X4 decoding) × GEAR_RATIO
    // → 90 × 4 × 1.0 = 360
    constexpr float MOTOR_BASE_PPR   = 90.0f;  // 360 / 4
    constexpr float MOTOR_GEAR_RATIO = 1.0f;   // Already measured at wheel output
    constexpr float WHEEL_DIA_MM     = 43.0f;

    // ==========================================
    // 4. BATTERY MONITOR (2S LiPo)
    // ==========================================
    // Using GPIO36 (VP) for ADC so it doesn't interfere with encoders
    constexpr adc_unit_t    BATT_ADC_UNIT = ADC_UNIT_1;
    constexpr adc_channel_t BATT_ADC_CHAN = ADC_CHANNEL_0; 
    
    // Voltage divider: 10k and 3.3k resistors -> (10 + 3.3) / 3.3 = ~4.03
    constexpr float BATT_DIV_MULTIPLIER = 4.03f;
    constexpr float BATT_MAX_VOLTAGE    = 8.4f;
    constexpr float BATT_MIN_VOLTAGE    = 6.4f;

    // ==========================================
    // 5. LINE SENSOR ARRAY (Sensor Fusion)
    // ==========================================
    constexpr uint16_t LINE_BLACK_THRESHOLD = 15000;
    // Weights correspond to physical sensor distance from the center
    constexpr int LINE_WEIGHTS[6] = {-2500, -1500, -500, 500, 1500, 2500};

    // ==========================================
    // 6. PID TUNING CONSTANTS
    // ==========================================
    constexpr float PID_KP           = 0.015f;
    constexpr float PID_KI           = 0.0001f;
    constexpr float PID_KD           = 0.005f;
    constexpr float PID_MIN_OUT      = -80.0f;
    constexpr float PID_MAX_OUT      = 80.0f;

} // namespace Config
