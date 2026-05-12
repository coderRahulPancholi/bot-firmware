#include "unity.h"
#include "robot_motors.h"
#include "hal/gpio_types.h"

// Bring in the generated Mocks!
#include "mock_pcf8574.h"
#include "mock_ledc.h"

void setUp(void) {}
void tearDown(void) {}

TEST_CASE("Steer hard right generates correct bitmask and speeds", "[motors]") {
    RobotMotors chassis(GPIO_NUM_18, GPIO_NUM_19);
    
    // We are testing: steer(50, 0); -> Pivot hard right in place.
    // Math: Left = 0 + 50 = +50 (Forward). Right = 0 - 50 = -50 (Reverse).
    // Left Forward Bit (0x01) | Right Reverse Bit (0x08) = 0x09
    
    // 1. SET THE EXPECTATIONS (What the mock hardware should see)
    // We expect I2C to write the 0x09 direction byte
    pcf8574_port_write_ExpectAndReturn(NULL, 0x09, ESP_OK);
    pcf8574_port_write_IgnoreArg_dev(); // We don't care about the exact device pointer here

    // We expect PWM Channel 0 (Left) to be set to duty 127 (50% of 255)
    ledc_set_duty_ExpectAndReturn(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 127, ESP_OK);
    ledc_update_duty_ExpectAndReturn(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, ESP_OK);

    // We expect PWM Channel 1 (Right) to also be set to duty 127 (50% of 255, abs value)
    ledc_set_duty_ExpectAndReturn(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 127, ESP_OK);
    ledc_update_duty_ExpectAndReturn(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ESP_OK);

    // 2. EXECUTE THE CODE
    chassis.steer(50, 0); 
    
    // 3. Unity will automatically FAIL the test if steer() didn't 
    // call the hardware exactly as we predicted above!
}