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
}

// Custom component includes
#include "system_monitor.h"
#include "robot_motors.h"
#include "line_sensors.h"
#include "pid_controller.h"

// Hardware config
#include "robot_config.h"
// Include user-generated Blockly code
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
// TELEMETRY TASK — streams JSON at 200ms
// =============================================
static void telemetryTask(void* pvParameters) {
    while (1) {
        float rpm_l, rpm_r, speed_l, speed_r;
        robot.getSpeeds(rpm_l, rpm_r, speed_l, speed_r);

        // Read all IR sensors
        sensors.readAll();

        // Build compact JSON telemetry packet
        printf("{\"device\":\"BotBuilder\",\"rpm1\":%.1f,\"rpm2\":%.1f,\"spd1\":%.3f,\"spd2\":%.3f,"
               "\"ir\":[%d,%d,%d,%d,%d,%d]}\n",
               rpm_l, rpm_r, speed_l, speed_r,
               sensors.getValue(0), sensors.getValue(1),
               sensors.getValue(2), sensors.getValue(3),
               sensors.getValue(4), sensors.getValue(5));

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
    if (esp_ble_conn_start() != ESP_OK) {
        esp_ble_conn_stop();
        esp_ble_conn_deinit();
        esp_event_handler_unregister(BLE_CONN_MGR_EVENTS, ESP_EVENT_ANY_ID, app_ble_conn_event_handler);
        esp_event_handler_unregister(BLE_CTS_EVENTS, ESP_EVENT_ANY_ID, app_ble_cts_event_handler);
    }

    // 1. Init LCD display
    monitor.init();

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
