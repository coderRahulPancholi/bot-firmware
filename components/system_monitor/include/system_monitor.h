#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_err.h"

// ESP-IDF LCD and SPI Headers
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

// --- SPI & LCD PINS ---
// Make sure these don't conflict with your Motors (18,19) or I2C (21,22)
#define LCD_HOST       SPI2_HOST
#define LCD_PIN_MOSI   GPIO_NUM_23
#define LCD_PIN_CLK    GPIO_NUM_18
#define LCD_PIN_CS     GPIO_NUM_15
#define LCD_PIN_DC     GPIO_NUM_2
#define LCD_PIN_RST    GPIO_NUM_4
#define LCD_PIN_BLK    GPIO_NUM_16 // Backlight

// ST7735 Resolution
#define LCD_H_RES 128
#define LCD_V_RES 160

struct ErrorAlert {
    char module[16];       
    char message[32];      
    esp_err_t error_code;  
};

class SystemMonitor {
private:
    static QueueHandle_t errorQueue;
    static esp_lcd_panel_handle_t panel_handle; // Global handle for the screen
    
    static void displayTask(void* pvParameters);
    
    // Core drawing methods
    static void drawErrorScreen(const ErrorAlert& alert);
    static void drawChar(int x, int y, char c, uint16_t color, uint16_t bg_color, uint16_t* buffer);
    static void drawString(int x, int y, const char* text, uint16_t color, uint16_t bg_color, uint16_t* buffer);

public:
    static esp_lcd_panel_handle_t getPanelHandle() { return panel_handle; }
    esp_err_t init();
    static void reportError(const char* module, const char* msg, esp_err_t err);
    void updateBatteryDisplay(float voltage, int percentage);
};