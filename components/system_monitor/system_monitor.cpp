#include "system_monitor.h"
#include "font8x8.h" 
#include "esp_log.h"
#include "esp_lcd_st7735.h" 
#include "esp_heap_caps.h"
#include <string.h>
#include <stdlib.h>
#include "esp_err.h" 

static const char* TAG = "MONITOR";

// Colors in RGB565 format
#define COLOR_RED   0xF800
#define COLOR_WHITE 0xFFFF
#define COLOR_BLACK 0x0000

QueueHandle_t SystemMonitor::errorQueue = nullptr;
esp_lcd_panel_handle_t SystemMonitor::panel_handle = nullptr;

esp_err_t SystemMonitor::init() {
    ESP_LOGI(TAG, "Initializing SPI Bus and LCD...");

    // 1. Create the Queue
    errorQueue = xQueueCreate(5, sizeof(ErrorAlert));
    if (errorQueue == NULL) return ESP_ERR_NO_MEM;

    // 2. Initialize the SPI Bus
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = LCD_PIN_MOSI;
    buscfg.miso_io_num = -1; // We only send data to the screen, we don't read
    buscfg.sclk_io_num = LCD_PIN_CLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.data4_io_num = -1;
    buscfg.data5_io_num = -1;
    buscfg.data6_io_num = -1;
    buscfg.data7_io_num = -1;
    buscfg.max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t);
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 3. Configure the Panel IO
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.cs_gpio_num = LCD_PIN_CS;
    io_config.dc_gpio_num = LCD_PIN_DC;
    io_config.spi_mode = 0;
    io_config.pclk_hz = 20 * 1000 * 1000; // 20 MHz
    io_config.trans_queue_depth = 10;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    // 4. Configure the ST7735 Driver
    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
    panel_config.bits_per_pixel = 16;
    panel_config.reset_gpio_num = LCD_PIN_RST;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7735(io_handle, &panel_config, &panel_handle));

    // 5. Turn it on
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    
    // Most 1.8" screens don't need a gap, and don't need inverted colors natively
    esp_lcd_panel_invert_color(panel_handle, false); 
    esp_lcd_panel_disp_on_off(panel_handle, true);

    // Wait 500ms for panel to wake up and stabilize before turning on backlight
    vTaskDelay(pdMS_TO_TICKS(500));

    // Turn on the backlight (PWM could be used here, but we just turn it to HIGH)
    gpio_set_direction(LCD_PIN_BLK, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_PIN_BLK, 1);

    // 6. Start the background task
    xTaskCreatePinnedToCore(displayTask, "Display_Task", 4096, NULL, 5, NULL, 1);

    return ESP_OK;
}

void SystemMonitor::reportError(const char* module, const char* msg, esp_err_t err) {
    // 1. ALWAYS print to the Serial Monitor first!
    // Using esp_err_to_name() translates the raw integer into a readable string (e.g., "ESP_ERR_TIMEOUT")
    ESP_LOGE("SYSTEM_FAULT", "🚨 [%s] %s | Code: %d (%s)", module, msg, err, esp_err_to_name(err));

    // 2. If the queue hasn't been created yet (or failed), we safely stop here.
    if (errorQueue == nullptr) return;

    // 3. Package the error for the LCD screen
    ErrorAlert alert;
    strncpy(alert.module, module, sizeof(alert.module) - 1);
    alert.module[sizeof(alert.module) - 1] = '\0'; // Ensure null-termination
    
    strncpy(alert.message, msg, sizeof(alert.message) - 1);
    alert.message[sizeof(alert.message) - 1] = '\0'; // Ensure null-termination
    
    alert.error_code = err;

    // 4. Send to the LCD Task
    xQueueSend(errorQueue, &alert, pdMS_TO_TICKS(10));
}

void SystemMonitor::displayTask(void* pvParameters) {
    ErrorAlert incoming_alert;

    while (1) {
        if (xQueueReceive(errorQueue, &incoming_alert, portMAX_DELAY)) {
            drawErrorScreen(incoming_alert);
        }
    }
}

// =========================================================================
// CUSTOM GRAPHICS ENGINE
// =========================================================================

void SystemMonitor::drawErrorScreen(const ErrorAlert& alert) {
    if (panel_handle == nullptr) return;

    // 1. Allocate a chunk of memory big enough to hold every pixel on the screen
    // We MUST use heap_caps_malloc with MALLOC_CAP_DMA because SPI requires DMA memory
    size_t buffer_size = LCD_H_RES * LCD_V_RES * sizeof(uint16_t);
    uint16_t* screen_buffer = (uint16_t*)heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);
    
    if (screen_buffer == NULL) {
        ESP_LOGE(TAG, "Not enough RAM to draw error screen!");
        return;
    }

    // 2. Flood the buffer with RED
    for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
        screen_buffer[i] = COLOR_RED;
    }

    // 3. Draw the text into the buffer
    // x, y, text, text color, background color, buffer
    drawString(10, 20, "SYSTEM FAULT", COLOR_WHITE, COLOR_RED, screen_buffer);
    drawString(10, 50, "MODULE:", COLOR_BLACK, COLOR_RED, screen_buffer);
    drawString(10, 65, alert.module, COLOR_WHITE, COLOR_RED, screen_buffer);
    
    drawString(10, 95, "ERROR:", COLOR_BLACK, COLOR_RED, screen_buffer);
    drawString(10, 110, alert.message, COLOR_WHITE, COLOR_RED, screen_buffer);

    // 4. Blast the entire buffer to the screen in one massive SPI transfer!
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, screen_buffer);

    // 5. Free the memory so the robot doesn't crash
    free(screen_buffer);
}

void SystemMonitor::drawString(int x, int y, const char* text, uint16_t color, uint16_t bg_color, uint16_t* buffer) {
    int cursor_x = x;
    while (*text) {
        char c = *text;
        // Convert lowercase to uppercase since our tiny font array only has caps
        if (c >= 'a' && c <= 'z') c -= 32; 
        
        drawChar(cursor_x, y, c, color, bg_color, buffer);
        cursor_x += 10; // Move 10 pixels to the right for the next letter
        text++;
    }
}

void SystemMonitor::drawChar(int x, int y, char c, uint16_t color, uint16_t bg_color, uint16_t* buffer) {
    // Check if character is in our font array bounds (32 to 90)
    if (c < 32 || c > 90) return; 

    // Fetch the 8 rows of pixels for this character
    const uint8_t* bitmap = font8x8_basic[c - 32];

    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            // Check if the specific bit is a 1 (draw text color) or 0 (draw background)
            if (bitmap[row] & (1 << (7 - col))) {
                // Calculate the 1D array position for this X,Y coordinate
                int pixel_index = ((y + row) * LCD_H_RES) + (x + col);
                // Ensure we don't draw outside the screen memory!
                if (pixel_index < (LCD_H_RES * LCD_V_RES)) {
                    buffer[pixel_index] = (color >> 8) | (color << 8); // Swap bytes for SPI
                }
            }
        }
    }
}

void SystemMonitor::updateBatteryDisplay(float voltage, int percentage) {
    char buffer[16];
    // Formats text like: " 8.1V  95%"
    snprintf(buffer, sizeof(buffer), "%4.1fV %3d%%", voltage, percentage);

    // 1. ALWAYS print to the Serial Monitor (Ultimate Fallback)
    ESP_LOGI("SYS_MONITOR", "Battery Status: %s", buffer);

    // 2. If the LCD is connected and initialized, draw it!
    if (panel_handle != nullptr) {
        
        // Allocate DMA memory for the screen buffer
        size_t buffer_size = LCD_H_RES * LCD_V_RES * sizeof(uint16_t);
        uint16_t* screen_buffer = (uint16_t*)heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);
        
        if (screen_buffer != NULL) {
            // Flood the background with Black to clear the previous frame
            for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
                screen_buffer[i] = COLOR_BLACK;
            }

            // Draw the telemetry text using your custom graphics engine
            // drawString(x, y, text, text_color, background_color, buffer)
            drawString(10, 10, "TELEMETRY", COLOR_WHITE, COLOR_BLACK, screen_buffer);
            
            drawString(10, 30, "BATTERY:", COLOR_WHITE, COLOR_BLACK, screen_buffer);
            drawString(10, 45, buffer, COLOR_RED, COLOR_BLACK, screen_buffer); // Make the voltage red!

            // Blast the frame to the LCD
            esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, LCD_H_RES, LCD_V_RES, screen_buffer);

            // Wait for SPI DMA transfer to finish (40KB @ 20MHz takes ~16ms)
            vTaskDelay(pdMS_TO_TICKS(30));

            // Free the memory so the ESP32 doesn't crash
            free(screen_buffer);
        } else {
            // If the ESP32 is out of RAM for some reason, warn the serial monitor
            ESP_LOGW(TAG, "Not enough DMA memory to update battery screen!");
        }
    }
}