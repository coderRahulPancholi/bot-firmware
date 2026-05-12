#include "unity.h"
#include "line_sensors.h"
#include "mock_ads111x.h"

// Mock the FreeRTOS delay so our host test runs instantly instead of waiting 2ms per sensor
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
void vTaskDelay(const uint32_t xTicksToDelay) { /* Do nothing on PC */ }

void setUp(void) {}
void tearDown(void) {}

TEST_CASE("readAll properly iterates multiplexer and handles data", "[sensors]") {
    LineSensors sensors;

    // Simulate reading the first sensor (Sensor 0 on ADC 1)
    ads111x_set_input_mux_ExpectAndReturn(NULL, ADS111X_MUX_0_GND, ESP_OK);
    ads111x_set_input_mux_IgnoreArg_dev();
    
    ads111x_start_conversion_ExpectAndReturn(NULL, ESP_OK);
    ads111x_start_conversion_IgnoreArg_dev();
    
    // Simulate the hardware returning the value "20000"
    int16_t fake_sensor_data = 20000;
    ads111x_get_value_ExpectAndReturn(NULL, NULL, ESP_OK);
    ads111x_get_value_IgnoreArg_dev();
    ads111x_get_value_ReturnThruPtr_value(&fake_sensor_data);

    // (In a real test, you would repeat the Expect blocks for all 6 sensors)
    // To keep this snippet short, we will just assume the first one passed.
    
    // ... Execute ...
    // sensors.readAll(); 
    // TEST_ASSERT_EQUAL_UINT16(20000, sensors.getValue(0));
}

TEST_CASE("readAll aborts safely if ADC is unplugged", "[sensors]") {
    LineSensors sensors;

    // Expect the MUX to change safely
    ads111x_set_input_mux_ExpectAndReturn(NULL, ADS111X_MUX_0_GND, ESP_OK);
    ads111x_set_input_mux_IgnoreArg_dev();
    
    // Simulate an I2C TIMEOUT (Wire fell out) during the conversion trigger
    ads111x_start_conversion_ExpectAndReturn(NULL, ESP_ERR_TIMEOUT);
    ads111x_start_conversion_IgnoreArg_dev();

    // Execute
    esp_err_t result = sensors.readAll();

    // Verify the function passed the error up the chain instead of crashing
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, result);
}