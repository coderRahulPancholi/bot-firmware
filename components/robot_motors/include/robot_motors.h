#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "i2cdev.h"
#include "pcf8574.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"

// Direction definitions for the PCF8574
#define DIR_STOP     0x00
#define DIR_FORWARD  0x05 // 0b00000101
#define DIR_BACKWARD 0x0A // 0b00001010
#define DIR_LEFT     0x06 // 0b00000110
#define DIR_RIGHT    0x09 // 0b00001001

class RobotMotors {
private:
    i2c_dev_t pcf_dev;
    
    // PWM configuration
    gpio_num_t pwm_pin_a;
    gpio_num_t pwm_pin_b;

    //speed pins
    uint32_t current_speed_a;
    uint32_t current_speed_b;

    // Encoder Pins
    gpio_num_t enc_a_left, enc_b_left;
    gpio_num_t enc_a_right, enc_b_right;

    // Hardware Pulse Counter Handles
    pcnt_unit_handle_t pcnt_unit_left;
    pcnt_unit_handle_t pcnt_unit_right;

    // Odometry Variables
    float ticks_per_rev;
    float wheel_circ_m;
    
    int last_enc_left;
    int last_enc_right;
    int64_t last_time_us;

    // Helper to configure ESP32 hardware PWM
    esp_err_t initPWM();
    esp_err_t initEncoders();

public:
    // Constructor
    RobotMotors(gpio_num_t pwm_a, gpio_num_t pwm_b, 
                gpio_num_t enc_a_l, gpio_num_t enc_b_l, 
                gpio_num_t enc_a_r, gpio_num_t enc_b_r);

    // Initialize the I2C expander and PWM timers
    esp_err_t init(i2c_port_t port, uint8_t pcf_address, gpio_num_t sda_pin, gpio_num_t scl_pin);

    // Movement commands
    void steer(int steering, int base_speed = 60);
    void moveForward();
    void moveBackward();
    void turnLeft();
    void turnRight();
    void stop();

    // Sets the physical properties of your robot for accurate math
    void setOdometryConfig(float base_ppr, float gear_ratio, float wheel_diameter_mm);

    // Calculates and returns the live RPM and meters/second for both wheels
    void getSpeeds(float &rpm_l, float &rpm_r, float &speed_l, float &speed_r);

    // Speed control (0 to 100 percent)
    void setSpeed(uint32_t speed_left, uint32_t speed_right);

    int getLeftEncoder();
    int getRightEncoder();
    void resetEncoders();
};