#include "robot_display.h"
#include "esp_log.h"
#include "robot_config.h"
#include "system_monitor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "font8x8.h"

static const char* TAG = "DISPLAY";

BotDisplay display;

static int16_t cursor_x = 0;
static int16_t cursor_y = 0;
static uint16_t text_color = 0xFFFF;
static uint8_t text_size = 1;
static uint8_t current_rotation = 0;

BotDisplay::BotDisplay() {}

esp_err_t BotDisplay::init() {
    ESP_LOGI(TAG, "Hardware Display Wrapper Init");
    return ESP_OK;
}

void BotDisplay::fillScreen(uint16_t color) {
    if (current_rotation == 1 || current_rotation == 3) {
        fillRect(0, 0, 160, 128, color);
    } else {
        fillRect(0, 0, 128, 160, color);
    }
}

void BotDisplay::setRotation(uint8_t r) {
    current_rotation = r & 3;
    // We handle all rotation mathematically in software!
    // ST7735 MADCTL hardware rotation is extremely buggy across different panels.
}

void BotDisplay::setCursor(int16_t x, int16_t y) {
    cursor_x = x;
    cursor_y = y;
}

void BotDisplay::setTextColor(uint16_t color) {
    text_color = color;
}

void BotDisplay::setTextSize(uint8_t size) {
    if (size == 0) size = 1;
    text_size = size;
}

void BotDisplay::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    esp_lcd_panel_handle_t panel = SystemMonitor::getPanelHandle();
    if (!panel || w <= 0 || h <= 0) return;
    
    // Software coordinate rotation for bounds
    int16_t rx = x, ry = y, rw = w, rh = h;
    switch (current_rotation) {
        case 1: rx = 128 - y - h; ry = x; rw = h; rh = w; break;
        case 2: rx = 128 - x - w; ry = 160 - y - h; break;
        case 3: rx = y; ry = 160 - x - w; rw = h; rh = w; break;
    }
    
    if (rx < 0 || ry < 0 || rx + rw > 128 || ry + rh > 160) return; // Prevent off-screen crashing
    
    // Allocate max 4KB for DMA buffer to prevent out-of-memory errors
    int max_lines = 4096 / (rw * sizeof(uint16_t));
    if (max_lines < 1) max_lines = 1;
    
    size_t size = rw * max_lines * sizeof(uint16_t);
    uint16_t* buf = (uint16_t*)heap_caps_malloc(size, MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "fillRect: No memory!");
        return;
    }
    
    uint16_t swapped = (color >> 8) | (color << 8);
    for(int i = 0; i < rw * max_lines; i++) buf[i] = swapped;
    
    for (int line = 0; line < rh; line += max_lines) {
        int lines_to_draw = (rh - line > max_lines) ? max_lines : (rh - line);
        esp_lcd_panel_draw_bitmap(panel, rx, ry + line, rx + rw, ry + line + lines_to_draw, buf);
        // Wait for SPI DMA to finish drawing this chunk (~2ms)
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    free(buf);
}

void BotDisplay::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    fillRect(x, y, w, 1, color);
    fillRect(x, y + h - 1, w, 1, color);
    fillRect(x, y, 1, h, color);
    fillRect(x + w - 1, y, 1, h, color);
}

void BotDisplay::print(const char* text) {
    esp_lcd_panel_handle_t panel = SystemMonitor::getPanelHandle();
    if (!panel) return;

    while (*text) {
        char c = *text;
        // Convert to uppercase since font array is limited
        if (c >= 'a' && c <= 'z') c -= 32; 
        
        if (c >= 32 && c <= 90) {
            int charIdx = c - 32;
            const uint8_t* bitmap = font8x8_basic[charIdx];
            
            int scale = text_size;
            int size = (8 * scale) * (8 * scale) * sizeof(uint16_t);
            uint16_t* buf = (uint16_t*)heap_caps_malloc(size, MALLOC_CAP_DMA);
            if (buf) {
                uint16_t swapped = (text_color >> 8) | (text_color << 8);
                // transparent background for now (black)
                for (int i=0; i < 64*scale*scale; i++) buf[i] = 0x0000;
                
                for (int row = 0; row < 8; row++) {
                    for (int col = 0; col < 8; col++) {
                        if (bitmap[row] & (1 << (7 - col))) {
                            // Software rotation of the character bitmap
                            int new_r = row;
                            int new_c = col;
                            switch (current_rotation) {
                                case 1: new_r = col; new_c = 7 - row; break;
                                case 2: new_r = 7 - row; new_c = 7 - col; break;
                                case 3: new_r = 7 - col; new_c = row; break;
                            }
                            
                            for (int sr=0; sr<scale; sr++) {
                                for (int sc=0; sc<scale; sc++) {
                                    int final_r = new_r * scale + sr;
                                    int final_c = new_c * scale + sc;
                                    buf[final_r * 8 * scale + final_c] = swapped;
                                }
                            }
                        }
                    }
                }
                
                // Software rotation of the cursor coordinates
                int16_t rx = cursor_x, ry = cursor_y;
                switch (current_rotation) {
                    case 1: rx = 128 - cursor_y - 8*scale; ry = cursor_x; break;
                    case 2: rx = 128 - cursor_x - 8*scale; ry = 160 - cursor_y - 8*scale; break;
                    case 3: rx = cursor_y; ry = 160 - cursor_x - 8*scale; break;
                }
                
                if (rx >= 0 && ry >= 0 && rx + 8*scale <= 128 && ry + 8*scale <= 160) {
                    esp_lcd_panel_draw_bitmap(panel, rx, ry, rx + 8*scale, ry + 8*scale, buf);
                    vTaskDelay(pdMS_TO_TICKS(2));
                }
                free(buf);
            }
        }
        cursor_x += 8 * text_size;
        text++;
    }
}

void BotDisplay::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    int16_t dx = abs(x1 - x0);
    int16_t dy = -abs(y1 - y0);
    int16_t sx = x0 < x1 ? 1 : -1;
    int16_t sy = y0 < y1 ? 1 : -1;
    int16_t err = dx + dy;
    
    while (true) {
        fillRect(x0, y0, 1, 1, color);
        if (x0 == x1 && y0 == y1) break;
        int16_t e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void BotDisplay::drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) { }
void BotDisplay::fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) { }
