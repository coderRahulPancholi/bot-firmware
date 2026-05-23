idf_component_register(
    SRCS "system_monitor.cpp"
    INCLUDE_DIRS "include" "."
    REQUIRES "waveshare__esp_lcd_st7735" "esp_lcd" "driver"
)