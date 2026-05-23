#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "i2cdev.h"
#include "hal/gpio_types.h"

extern "C" {
#include "nvs_flash.h"
#include "esp_ble_conn_mgr.h"
#include "esp_cts.h"
#include "esp_ota_ops.h"
#include "esp_ble_ota_svc.h"
}

// Custom component includes
#include "system_monitor.h"
#include "robot_motors.h"
#include "line_sensors.h"
#include "pid_controller.h"

// Hardware config
#include "robot_config.h"
#include "robot_display.h"
// Include user-generated Blockly code
// Forward declarations for Blockly generated code
void delay(uint32_t ms);
void setMotors(int speedLeft, int speedRight);
#include "user_logic.h"

static const char* TAG = "BOT_IDF";

static void app_ble_cts_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_ble_cts_cur_time_t *cur_time = NULL;
    esp_ble_cts_local_time_t *local_time = NULL;
    esp_ble_cts_ref_time_t *ref_time = NULL;

    if ((base != BLE_CTS_EVENTS) || (event_data == NULL)) {
        return;
    }

    switch (id) {
#ifdef CONFIG_BLE_CTS_CURRENT_TIME_WRITE_ENABLE
    case BLE_CTS_CHR_UUID16_CURRENT_TIME:
        cur_time = (esp_ble_cts_cur_time_t *)event_data;
        ESP_LOGI(TAG, "Current time, year = %d, month = %d", cur_time->timestamp.year, cur_time->timestamp.month);
        break;
#endif
#if defined(CONFIG_BLE_CTS_LOCAL_TIME_CHAR_ENABLE) && defined(CONFIG_BLE_CTS_LOCAL_TIME_WRITE_ENABLE)
    case BLE_CTS_CHR_UUID16_LOCAL_TIME:
        local_time = (esp_ble_cts_local_time_t *)event_data;
        ESP_LOGI(TAG, "Local time, timezone = %d", local_time->timezone);
        break;
#endif
#ifdef CONFIG_BLE_CTS_REF_TIME_CHAR_ENABLE
    case BLE_CTS_CHR_UUID16_REFERENCE_TIME:
        ref_time = (esp_ble_cts_ref_time_t *)event_data;
        ESP_LOGI(TAG, "Reference time, time accuracy = %d", ref_time->time_accuracy);
        break;
#endif
    default:
        break;
    }
}

static void app_ble_cts_init(void)
{
    esp_ble_cts_init();
    esp_event_handler_register(BLE_CTS_EVENTS, ESP_EVENT_ANY_ID, app_ble_cts_event_handler, NULL);
}

static void app_ble_conn_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_ble_cts_cur_time_t cur_time = {};
    cur_time.timestamp.year = 2024;
    cur_time.timestamp.month = 10;
    cur_time.timestamp.day = 1;
    cur_time.timestamp.hours = 9;
    cur_time.timestamp.minutes = 10;
    cur_time.timestamp.seconds = 25;
    cur_time.day_of_week = 1;
    cur_time.fractions_256 = 6;
    cur_time.adjust_reason = BLE_CTS_MANUAL_TIME_UPDATE_MASK;

    if (base != BLE_CONN_MGR_EVENTS) {
        return;
    }

    switch (id) {
    case ESP_BLE_CONN_EVENT_CONNECTED:
        ESP_LOGI(TAG, "ESP_BLE_CONN_EVENT_CONNECTED");
        esp_ble_cts_set_current_time(&cur_time, true);
        break;
    case ESP_BLE_CONN_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "ESP_BLE_CONN_EVENT_DISCONNECTED");
        break;
    default:
        break;
    }
}

// --- Custom BLE UART Service (NUS) ---
#define NUS_SERVICE_UUID {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E}
#define NUS_RX_CHAR_UUID {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E}
#define NUS_TX_CHAR_UUID {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E}

static esp_err_t uart_rx_cb(const uint8_t *inbuf, uint16_t inlen,
                            uint8_t **outbuf, uint16_t *outlen, void *priv_data, uint8_t *att_status)
{
    if (inbuf && inlen > 0) {
        ESP_LOGI(TAG, "UART RX: %.*s", inlen, inbuf);
    }
    *att_status = ESP_IOT_ATT_SUCCESS;
    return ESP_OK;
}

static esp_err_t uart_tx_cb(const uint8_t *inbuf, uint16_t inlen,
                            uint8_t **outbuf, uint16_t *outlen, void *priv_data, uint8_t *att_status)
{
    *att_status = ESP_IOT_ATT_SUCCESS;
    return ESP_OK;
}

static const esp_ble_conn_character_t uart_nu_lookup_table[] = {
    {
        "UART RX", BLE_CONN_UUID_TYPE_128, BLE_CONN_GATT_CHR_WRITE | BLE_CONN_GATT_CHR_WRITE_NO_RSP,
        { .uuid128 = NUS_RX_CHAR_UUID }, uart_rx_cb
    },
    {
        "UART TX", BLE_CONN_UUID_TYPE_128, BLE_CONN_GATT_CHR_READ | BLE_CONN_GATT_CHR_NOTIFY,
        { .uuid128 = NUS_TX_CHAR_UUID }, uart_tx_cb
    }
};

static const esp_ble_conn_svc_t uart_svc = {
    .type = BLE_CONN_UUID_TYPE_128,
    .nu_lookup_count = sizeof(uart_nu_lookup_table) / sizeof(uart_nu_lookup_table[0]),
    .uuid = { .uuid128 = NUS_SERVICE_UUID },
    .nu_lookup = (esp_ble_conn_character_t *)uart_nu_lookup_table
};

static void app_ble_uart_init(void)
{
    esp_ble_conn_add_svc(&uart_svc);
}

// Function to send telemetry over BLE Notify
void ble_uart_send(const char *data) {
    uint16_t len = strlen(data);
    esp_ble_conn_data_t conn_data = {};
    conn_data.type = BLE_CONN_UUID_TYPE_128;
    memcpy(conn_data.uuid.uuid128, (uint8_t[])NUS_TX_CHAR_UUID, 16);
    conn_data.data = (uint8_t *)data;
    conn_data.data_len = len;
    esp_ble_conn_notify(&conn_data);
}

// --- OTA Implementation ---
static esp_ota_handle_t update_handle = 0;
static const esp_partition_t *update_partition = NULL;
static size_t ota_total_bytes = 0;

extern "C" void esp_ble_ota_recv_cmd_data(const uint8_t *data, uint16_t len) {
    if (len == 0) return;
    if (data[0] == 0x01) { // START OTA
        ESP_LOGI(TAG, "OTA Begin");
        update_partition = esp_ota_get_next_update_partition(NULL);
        if (update_partition == NULL) {
            ESP_LOGE(TAG, "No OTA partition found");
            return;
        }
        esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
            return;
        }
        ota_total_bytes = 0;
        esp_ble_ota_notify_command_raw((const uint8_t*)"OK", 2);
    } else if (data[0] == 0x02) { // END OTA
        ESP_LOGI(TAG, "OTA End");
        esp_err_t err = esp_ota_end(update_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_end failed!");
            return;
        }
        err = esp_ota_set_boot_partition(update_partition);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_set_boot_partition failed!");
            return;
        }
        esp_ble_ota_notify_command_raw((const uint8_t*)"OK", 2);
        ESP_LOGI(TAG, "OTA Success, restarting in 1s...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
}

extern "C" void esp_ble_ota_recv_fw_data(const uint8_t *data, uint16_t len) {
    if (update_handle != 0) {
        esp_err_t err = esp_ota_write(update_handle, data, len);
        if (err == ESP_OK) {
            ota_total_bytes += len;
            // Send ACK with received byte count
            uint8_t ack[4];
            ack[0] = (ota_total_bytes >> 24) & 0xFF;
            ack[1] = (ota_total_bytes >> 16) & 0xFF;
            ack[2] = (ota_total_bytes >> 8) & 0xFF;
            ack[3] = ota_total_bytes & 0xFF;
            esp_ble_ota_notify_recv_fw_raw(ack, 4);
        } else {
            ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
        }
    }
}

// --- Global Objects (same pattern as line_following_robot) ---
SystemMonitor monitor;

RobotMotors robot(
    Config::MOTOR_PWM_A, Config::MOTOR_PWM_B,
    Config::ENC_L_A, Config::ENC_L_B,
    Config::ENC_R_A, Config::ENC_R_B
);

LineSensors sensors;

PIDController linePID(
    Config::PID_KP, Config::PID_KI, Config::PID_KD,
    Config::PID_MIN_OUT, Config::PID_MAX_OUT
);

// --- BLOCKLY COMPATIBILITY WRAPPERS ---
void delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void setMotors(int speedLeft, int speedRight) {
    uint32_t pwmLeft  = (abs(speedLeft)  * 100) / 255;
    uint32_t pwmRight = (abs(speedRight) * 100) / 255;
    robot.setSpeed(pwmLeft, pwmRight);

    if      (speedLeft > 0 && speedRight > 0) robot.moveForward();
    else if (speedLeft < 0 && speedRight < 0) robot.moveBackward();
    else if (speedLeft < 0 && speedRight > 0) robot.turnLeft();
    else if (speedLeft > 0 && speedRight < 0) robot.turnRight();
    else                                       robot.stop();
}

// =============================================
// DISPLAY UI FUNCTIONS
// =============================================
static bool ui_initialized = false;

static void drawStaticUI() {
#ifdef USER_ROTATION
    display.setRotation(USER_ROTATION);
#else
    display.setRotation(1); 
#endif

#ifdef USER_BG_COLOR
    display.fillScreen(USER_BG_COLOR);
#else
    display.fillScreen(Config::COLOR_BLACK);
#endif
    
    display.setTextColor(Config::COLOR_CYAN);
    display.setTextSize(1);
    display.setCursor(50, 5);
    display.print("IR SENSORS");
    
    display.drawLine(0, 15, 160, 15, Config::COLOR_WHITE);
    
    display.setTextColor(Config::COLOR_YELLOW);
    display.setCursor(5, 20); display.print("S1:"); 
    display.setCursor(5, 35); display.print("S2:"); 
    display.setCursor(5, 50); display.print("S3:");
    
    display.setCursor(85, 20); display.print("S4:"); 
    display.setCursor(85, 35); display.print("S5:"); 
    display.setCursor(85, 50); display.print("S6:");
    
    display.drawLine(0, 70, 160, 70, Config::COLOR_WHITE);
    
    display.setTextColor(Config::COLOR_CYAN);
    display.setCursor(50, 75); display.print("MOTOR RPM");
    
    display.drawLine(0, 85, 160, 85, Config::COLOR_WHITE);
    
    display.setTextColor(Config::COLOR_GREEN);
    display.setCursor(5, 95); display.print("M1 (L):");
    display.setCursor(5, 110); display.print("M2 (R):");
}

static void updateSensorUI(int16_t s0, int16_t s1, int16_t s2, int16_t s3, int16_t s4, int16_t s5) {
    display.setTextSize(1); 
    display.setTextColor(Config::COLOR_WHITE); 
    
    // Clear old text by drawing a black rectangle over the values
    display.fillRect(25, 20, 40, 8, Config::COLOR_BLACK);
    display.fillRect(25, 35, 40, 8, Config::COLOR_BLACK);
    display.fillRect(25, 50, 40, 8, Config::COLOR_BLACK);
    
    display.fillRect(105, 20, 40, 8, Config::COLOR_BLACK);
    display.fillRect(105, 35, 40, 8, Config::COLOR_BLACK);
    display.fillRect(105, 50, 40, 8, Config::COLOR_BLACK);

    char buf[16];
    sprintf(buf, "%d", s0); display.setCursor(25, 20); display.print(buf);
    sprintf(buf, "%d", s1); display.setCursor(25, 35); display.print(buf);
    sprintf(buf, "%d", s2); display.setCursor(25, 50); display.print(buf);
    
    sprintf(buf, "%d", s3); display.setCursor(105, 20); display.print(buf);
    sprintf(buf, "%d", s4); display.setCursor(105, 35); display.print(buf);
    sprintf(buf, "%d", s5); display.setCursor(105, 50); display.print(buf);
}

static void updateMotorUI(float rpm_l, float rpm_r) {
    display.setTextSize(1); 
    display.setTextColor(Config::COLOR_WHITE); 
    
    display.fillRect(55, 95, 60, 8, Config::COLOR_BLACK);
    display.fillRect(55, 110, 60, 8, Config::COLOR_BLACK);
    
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", rpm_l); display.setCursor(55, 95); display.print(buf);
    snprintf(buf, sizeof(buf), "%.1f", rpm_r); display.setCursor(55, 110); display.print(buf);
}

// =============================================
// TELEMETRY TASK — streams JSON at 200ms
// =============================================
static void telemetryTask(void* pvParameters) {
    while (1) {
        if (!ui_initialized) {
            drawStaticUI();
            ui_initialized = true;
        }

        float rpm_l, rpm_r, speed_l, speed_r;
        robot.getSpeeds(rpm_l, rpm_r, speed_l, speed_r);

        // Read all IR sensors
        sensors.readAll();

        // Update LCD Display with Arduino layout
        updateSensorUI(
            sensors.getValue(0), sensors.getValue(1), sensors.getValue(2),
            sensors.getValue(3), sensors.getValue(4), sensors.getValue(5)
        );
        updateMotorUI(rpm_l, rpm_r);

        // Build compact JSON telemetry packet
        char telemetry_buf[128];
        snprintf(telemetry_buf, sizeof(telemetry_buf),
               "{\"device\":\"BotBuilder\",\"rpm1\":%.1f,\"rpm2\":%.1f,\"spd1\":%.3f,\"spd2\":%.3f,"
               "\"ir\":[%d,%d,%d,%d,%d,%d]}\n",
               rpm_l, rpm_r, speed_l, speed_r,
               sensors.getValue(0), sensors.getValue(1),
               sensors.getValue(2), sensors.getValue(3),
               sensors.getValue(4), sensors.getValue(5));

        // Print to serial
        printf("%s", telemetry_buf);
        // Send over BLE
        ble_uart_send(telemetry_buf);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// =============================================
// USER SEQUENCE TASK — runs Blockly code once
// =============================================
static void userSequenceTask(void* pvParameters) {
    ESP_LOGI(TAG, "Running user sequence...");
    run_user_sequence();
    ESP_LOGI(TAG, "User sequence complete.");
    robot.stop();
    vTaskDelete(NULL); // Self-delete when done
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Booting BotBuilder Firmware v2.0...");

    // Initialize NVS (needed for BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_event_loop_create_default();
    esp_event_handler_register(BLE_CONN_MGR_EVENTS, ESP_EVENT_ANY_ID, app_ble_conn_event_handler, NULL);

    esp_ble_conn_config_t ble_config = {};
    snprintf((char *)ble_config.device_name, sizeof(ble_config.device_name), "BotBuilder_BLE");
    snprintf((char *)ble_config.broadcast_data, sizeof(ble_config.broadcast_data), "Bot_Adv");

    esp_ble_conn_init(&ble_config);
    app_ble_cts_init();
    app_ble_uart_init(); // Initialize UART GATT Service
    esp_ble_ota_svc_init(); // Initialize OTA GATT Service

    if (esp_ble_conn_start() != ESP_OK) {
        esp_ble_conn_stop();
        esp_ble_conn_deinit();
        esp_event_handler_unregister(BLE_CONN_MGR_EVENTS, ESP_EVENT_ANY_ID, app_ble_conn_event_handler);
        esp_event_handler_unregister(BLE_CTS_EVENTS, ESP_EVENT_ANY_ID, app_ble_cts_event_handler);
    }

    // 1. Init LCD display
    monitor.init();
    display.init();

#ifdef USER_ROTATION
    display.setRotation(USER_ROTATION);
#endif
#ifdef USER_TEXT_SIZE
    display.setTextSize(USER_TEXT_SIZE);
#endif
#ifdef USER_TEXT_COLOR
    display.setTextColor(USER_TEXT_COLOR);
#endif
#ifdef USER_BG_COLOR
    display.fillScreen(USER_BG_COLOR);
#endif
    // 2. Init I2C bus
    ESP_ERROR_CHECK(i2cdev_init());

    // 3. Init Motors
    esp_err_t err_motors = robot.init(
        Config::I2C_PORT, Config::PCF8574_ADDR,
        Config::I2C_SDA_PIN, Config::I2C_SCL_PIN
    );
    if (err_motors != ESP_OK) {
        SystemMonitor::reportError("MOTORS", "Init Failed", err_motors);
    }
    robot.setOdometryConfig(Config::MOTOR_BASE_PPR, Config::MOTOR_GEAR_RATIO, Config::WHEEL_DIA_MM);

    // 4. Init Line Sensors
    esp_err_t err_sensors = sensors.init(
        Config::I2C_PORT,
        Config::I2C_SDA_PIN, Config::I2C_SCL_PIN,
        Config::ADS1115_LEFT, Config::ADS1115_RIGHT
    );
    if (err_sensors != ESP_OK) {
        SystemMonitor::reportError("SENSORS", "Init Failed", err_sensors);
    }

    // 5. Show boot status on LCD
    monitor.updateBatteryDisplay(8.4f, 100);

    // 6. Start background telemetry stream (Core 0)
    xTaskCreatePinnedToCore(telemetryTask, "Telemetry", 4096, NULL, 3, NULL, 0);

    // 7. Run user Blockly sequence (Core 1)
    xTaskCreatePinnedToCore(userSequenceTask, "UserSeq", 8192, NULL, 5, NULL, 1);
}
