# Register the test file
idf_component_register(SRCS "test_motors.cpp"
                       REQUIRES "unity" "robot_motors")

# Tell ESP-IDF to generate mock versions of our hardware drivers!
idf_component_mock(REQUIRES "esp-idf-lib" MOCK_HEADER "pcf8574.h")
idf_component_mock(REQUIRES "driver" MOCK_HEADER "driver/ledc.h")