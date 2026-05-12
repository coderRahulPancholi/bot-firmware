idf_component_register(
    SRCS "robot_motors.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "esp_driver_ledc" "esp_driver_pcnt" "driver" "pcf8574" "system_monitor" "esp_timer"
)