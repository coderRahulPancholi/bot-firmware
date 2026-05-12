#include "esp_timer.h"
#include <stdlib.h>
#include "robot_motors.h"
#include "system_monitor.h" 
#include "esp_log.h"

static const char *TAG = "MOTORS";

RobotMotors::RobotMotors(gpio_num_t pwm_a, gpio_num_t pwm_b, 
                         gpio_num_t enc_a_l, gpio_num_t enc_b_l, 
                         gpio_num_t enc_a_r, gpio_num_t enc_b_r) {
    this->pwm_pin_a = pwm_a;
    this->pwm_pin_b = pwm_b;
    this->enc_a_left = enc_a_l;
    this->enc_b_left = enc_b_l;
    this->enc_a_right = enc_a_r;
    this->enc_b_right = enc_b_r;
    this->current_speed_a = 0;
    this->current_speed_b = 0;
}

esp_err_t RobotMotors::initPWM() {
    esp_err_t err;

    // 1. Configure the PWM Timer (5kHz frequency, 8-bit resolution)
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.duty_resolution = LEDC_TIMER_8_BIT; // 0-255 duty cycle
    timer_conf.timer_num = LEDC_TIMER_0;
    timer_conf.freq_hz = 5000;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;

    err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        SystemMonitor::reportError("MOTORS", "PWM Timer Config Fail", err);
        return err;
    }

    // 2. Configure Channel for Motor A (Left)
    ledc_channel_config_t ch_a_conf = {};
    ch_a_conf.gpio_num = pwm_pin_a;
    ch_a_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_a_conf.channel = LEDC_CHANNEL_0;
    ch_a_conf.timer_sel = LEDC_TIMER_0;
    ch_a_conf.duty = 0;
    ch_a_conf.hpoint = 0;

    err = ledc_channel_config(&ch_a_conf);
    if (err != ESP_OK) {
        SystemMonitor::reportError("MOTORS", "PWM Ch_A Config Fail", err);
        return err;
    }

    // 3. Configure Channel for Motor B (Right)
    ledc_channel_config_t ch_b_conf = {};
    ch_b_conf.gpio_num = pwm_pin_b;
    ch_b_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    ch_b_conf.channel = LEDC_CHANNEL_1;
    ch_b_conf.timer_sel = LEDC_TIMER_0;
    ch_b_conf.duty = 0;
    ch_b_conf.hpoint = 0;

    err = ledc_channel_config(&ch_b_conf);
    if (err != ESP_OK) {
        SystemMonitor::reportError("MOTORS", "PWM Ch_B Config Fail", err);
    }
    
    return err;
}

esp_err_t RobotMotors::initEncoders() {
    esp_err_t err;
    
    // 1. Configure the PCNT Units (Zero-initialized first to satisfy the strict compiler!)
    pcnt_unit_config_t unit_config = {};
    unit_config.low_limit = -32768;
    unit_config.high_limit = 32767;
    
    err = pcnt_new_unit(&unit_config, &pcnt_unit_left);
    if (err != ESP_OK) return err;
    err = pcnt_new_unit(&unit_config, &pcnt_unit_right);
    if (err != ESP_OK) return err;

    // 2. Add Glitch Filters (Crucial for physical motors)
    pcnt_glitch_filter_config_t filter_config = {};
    filter_config.max_glitch_ns = 1000;
    pcnt_unit_set_glitch_filter(pcnt_unit_left, &filter_config);
    pcnt_unit_set_glitch_filter(pcnt_unit_right, &filter_config);

    // 3. Setup Left Motor Channels
    pcnt_chan_config_t chan_a_left_cfg = {};
    chan_a_left_cfg.edge_gpio_num = enc_a_left;
    chan_a_left_cfg.level_gpio_num = enc_b_left;
    pcnt_channel_handle_t chan_a_left;
    pcnt_new_channel(pcnt_unit_left, &chan_a_left_cfg, &chan_a_left);
    
    pcnt_chan_config_t chan_b_left_cfg = {};
    chan_b_left_cfg.edge_gpio_num = enc_b_left;
    chan_b_left_cfg.level_gpio_num = enc_a_left;
    pcnt_channel_handle_t chan_b_left;
    pcnt_new_channel(pcnt_unit_left, &chan_b_left_cfg, &chan_b_left);

    // 4. Setup Right Motor Channels
    pcnt_chan_config_t chan_a_right_cfg = {};
    chan_a_right_cfg.edge_gpio_num = enc_a_right;
    chan_a_right_cfg.level_gpio_num = enc_b_right;
    pcnt_channel_handle_t chan_a_right;
    pcnt_new_channel(pcnt_unit_right, &chan_a_right_cfg, &chan_a_right);
    
    pcnt_chan_config_t chan_b_right_cfg = {};
    chan_b_right_cfg.edge_gpio_num = enc_b_right;
    chan_b_right_cfg.level_gpio_num = enc_a_right;
    pcnt_channel_handle_t chan_b_right;
    pcnt_new_channel(pcnt_unit_right, &chan_b_right_cfg, &chan_b_right);

    // 5. Configure Quadrature Logic (Hardware decoding for forward/reverse)
    pcnt_channel_set_edge_action(chan_a_left, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(chan_a_left, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(chan_b_left, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chan_b_left, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_channel_set_edge_action(chan_a_right, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(chan_a_right, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(chan_b_right, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chan_b_right, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    // 6. Enable and Start the counters
    pcnt_unit_enable(pcnt_unit_left);
    pcnt_unit_enable(pcnt_unit_right);
    pcnt_unit_clear_count(pcnt_unit_left);
    pcnt_unit_clear_count(pcnt_unit_right);
    pcnt_unit_start(pcnt_unit_left);
    pcnt_unit_start(pcnt_unit_right);

    return ESP_OK;
}

esp_err_t RobotMotors::init(i2c_port_t port, uint8_t pcf_address, gpio_num_t sda_pin, gpio_num_t scl_pin) {
    ESP_LOGI(TAG, "Initializing Motor Controller...");
    esp_err_t err;

    // Initialize the PCF8574
    err = pcf8574_init_desc(
        &pcf_dev,
        pcf_address,
        port,
        static_cast<gpio_num_t>(sda_pin),
        static_cast<gpio_num_t>(scl_pin)
    );
    
    if (err != ESP_OK) {
        SystemMonitor::reportError("MOTORS", "PCF8574 Init Fail", err);
        return err;
    }

    // Initialize ESP32 PWM
    err = initPWM();
    if (err != ESP_OK) {
        return err; // Error was already reported inside initPWM()
    }

    err = initEncoders();
    if (err != ESP_OK) {
        SystemMonitor::reportError("MOTORS", "PCNT Encoder Init Fail", err);
        return err;
    }

    // Ensure motors are stopped on boot
    err = pcf8574_port_write(&pcf_dev, DIR_STOP);
    if (err != ESP_OK) {
        SystemMonitor::reportError("MOTORS", "I2C Write Fail on Boot", err);
        return err;
    }

    ESP_LOGI(TAG, "Motor Controller Ready.");
    return ESP_OK;
}

void RobotMotors::steer(int steering, int base_speed) {
    // 1. Calculate ideal motor speeds (Differential Mixing)
    int left_speed = base_speed + steering;
    int right_speed = base_speed - steering;

    // 2. Constrain speeds to physical PWM limits (-100% to 100%)
    if (left_speed > 100) left_speed = 100;
    if (left_speed < -100) left_speed = -100;
    
    if (right_speed > 100) right_speed = 100;
    if (right_speed < -100) right_speed = -100;

    // 3. Dynamically build the PCF8574 direction byte
    uint8_t dir_byte = 0;

    if (left_speed > 0) {
        dir_byte |= 0b00000001; 
    } else if (left_speed < 0) {
        dir_byte |= 0b00000010; 
    }

    if (right_speed > 0) {
        dir_byte |= 0b00000100; 
    } else if (right_speed < 0) {
        dir_byte |= 0b00001000; 
    }

    // 4. Fire the commands to the hardware
    esp_err_t err = pcf8574_port_write(&pcf_dev, dir_byte);
    if (err != ESP_OK) {
        // HOT PATH FAULT DETECTION
        SystemMonitor::reportError("MOTORS", "I2C Write Fail (Steer)", err);
    }
    
    setSpeed(abs(left_speed), abs(right_speed));
}

void RobotMotors::moveForward() {
    esp_err_t err = pcf8574_port_write(&pcf_dev, DIR_FORWARD);
    if (err != ESP_OK) SystemMonitor::reportError("MOTORS", "I2C Write Fail (Fwd)", err);
}

void RobotMotors::moveBackward() {
    esp_err_t err = pcf8574_port_write(&pcf_dev, DIR_BACKWARD);
    if (err != ESP_OK) SystemMonitor::reportError("MOTORS", "I2C Write Fail (Rev)", err);
}

void RobotMotors::turnLeft() {
    esp_err_t err = pcf8574_port_write(&pcf_dev, DIR_LEFT);
    if (err != ESP_OK) SystemMonitor::reportError("MOTORS", "I2C Write Fail (Left)", err);
}

void RobotMotors::turnRight() {
    esp_err_t err = pcf8574_port_write(&pcf_dev, DIR_RIGHT);
    if (err != ESP_OK) SystemMonitor::reportError("MOTORS", "I2C Write Fail (Right)", err);
}

void RobotMotors::stop() {
    esp_err_t err = pcf8574_port_write(&pcf_dev, DIR_STOP);
    if (err != ESP_OK) SystemMonitor::reportError("MOTORS", "I2C Write Fail (Stop)", err);
    setSpeed(0, 0); // Cut power as well
}

void RobotMotors::setSpeed(uint32_t speed_left, uint32_t speed_right) {
    // Constrain to 100%
    if (speed_left > 100) speed_left = 100;
    if (speed_right > 100) speed_right = 100;

    // Convert 0-100% to 0-255 (8-bit resolution)
    uint32_t duty_a = (speed_left * 255) / 100;
    uint32_t duty_b = (speed_right * 255) / 100;

    // Update Left Motor
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_a);
    esp_err_t err = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    if (err != ESP_OK) SystemMonitor::reportError("MOTORS", "PWM Update Fail (Left)", err);

    // Update Right Motor
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_b);
    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    if (err != ESP_OK) SystemMonitor::reportError("MOTORS", "PWM Update Fail (Right)", err);
}

int RobotMotors::getLeftEncoder() {
    int count = 0;
    esp_err_t err = pcnt_unit_get_count(pcnt_unit_left, &count);
    if (err != ESP_OK) {
        SystemMonitor::reportError("MOTORS", "Left Encoder Read Fail", err);
    }
    return count;
}

int RobotMotors::getRightEncoder() {
    int count = 0;
    esp_err_t err = pcnt_unit_get_count(pcnt_unit_right, &count);
    if (err != ESP_OK) {
        SystemMonitor::reportError("MOTORS", "Right Encoder Read Fail", err);
    }
    return count;
}

void RobotMotors::resetEncoders() {
    esp_err_t err1 = pcnt_unit_clear_count(pcnt_unit_left);
    esp_err_t err2 = pcnt_unit_clear_count(pcnt_unit_right);
    
    if (err1 != ESP_OK || err2 != ESP_OK) {
        SystemMonitor::reportError("MOTORS", "Encoder Reset Failed", err1 != ESP_OK ? err1 : err2);
    }
}

void RobotMotors::setOdometryConfig(float base_ppr, float gear_ratio, float wheel_diameter_mm) {
    // Calculate total ticks per wheel revolution using X4 hardware decoding
    this->ticks_per_rev = (base_ppr * 4.0f) * gear_ratio;
    
    // Calculate wheel circumference in meters ( Pi * D )
    this->wheel_circ_m = (wheel_diameter_mm / 1000.0f) * 3.14159f;

    // Initialize baseline tracking variables
    this->last_enc_left = getLeftEncoder();
    this->last_enc_right = getRightEncoder();
    this->last_time_us = esp_timer_get_time();
}

void RobotMotors::getSpeeds(float &rpm_l, float &rpm_r, float &speed_l, float &speed_r) {
    // 1. Grab current time and current ticks
    int64_t current_time_us = esp_timer_get_time();
    int current_enc_left = getLeftEncoder();
    int current_enc_right = getRightEncoder();

    // 2. Calculate the Deltas (Change since last check)
    int delta_left = current_enc_left - last_enc_left;
    int delta_right = current_enc_right - last_enc_right;
    
    // Time delta in Minutes (microseconds / 60,000,000)
    float delta_time_min = (float)(current_time_us - last_time_us) / 60000000.0f;

    // Safety check to prevent division by zero if called too rapidly
    if (delta_time_min <= 0.0f) {
        rpm_l = 0; rpm_r = 0; speed_l = 0; speed_r = 0;
        return;
    }

    // 3. Math: RPM = (Delta Ticks / Ticks per Rev) / Time in Minutes
    rpm_l = ((float)delta_left / ticks_per_rev) / delta_time_min;
    rpm_r = ((float)delta_right / ticks_per_rev) / delta_time_min;

    // 4. Math: Linear Speed (m/s) = (RPM / 60) * Circumference
    speed_l = (rpm_l / 60.0f) * wheel_circ_m;
    speed_r = (rpm_r / 60.0f) * wheel_circ_m;

    // 5. Save the current state for the next time this function is called
    last_enc_left = current_enc_left;
    last_enc_right = current_enc_right;
    last_time_us = current_time_us;
}