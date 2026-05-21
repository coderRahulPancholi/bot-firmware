#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "i2cdev.h"
#include "hal/gpio_types.h"

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
