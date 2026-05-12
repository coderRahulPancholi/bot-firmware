idf_component_register(
    SRCS "mpu6050_imu.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "mpu6050" "system_monitor"
)