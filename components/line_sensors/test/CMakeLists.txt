idf_component_register(SRCS "test_sensors.cpp"
                       REQUIRES "unity" "line_sensors")

idf_component_mock(REQUIRES "esp-idf-lib" MOCK_HEADER "ads111x.h")