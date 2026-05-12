idf_component_register(
    SRCS "line_sensors.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "ads111x" "system_monitor"
)