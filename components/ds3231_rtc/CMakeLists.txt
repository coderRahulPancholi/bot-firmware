idf_component_register(
    SRCS "ds3231_rtc.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "ds3231" "system_monitor"
)