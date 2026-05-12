#include "pid_controller.h"
#include "esp_timer.h" // ESP-IDF high-resolution timer

PIDController::PIDController(float p, float i, float d, float min_out, float max_out) {
    this->kp = p;
    this->ki = i;
    this->kd = d;
    this->output_min = min_out;
    this->output_max = max_out;
    
    reset(); // Initialize memory variables to 0
}

void PIDController::setTunings(float p, float i, float d) {
    this->kp = p;
    this->ki = i;
    this->kd = d;
}

void PIDController::reset() {
    this->integral = 0.0f;
    this->previous_error = 0.0f;
    this->last_time_us = 0;
    this->first_run = true;
}

float PIDController::compute(float setpoint, float measured_value) {
    // 1. Calculate precise time delta (dt) in seconds
    int64_t now_us = esp_timer_get_time();
    
    if (first_run) {
        last_time_us = now_us;
        previous_error = setpoint - measured_value;
        first_run = false;
        return 0.0f; // Need one loop to establish a time baseline
    }

    float dt = (float)(now_us - last_time_us) / 1000000.0f; // Convert microseconds to seconds
    last_time_us = now_us;

    // Safety check to prevent divide-by-zero if called impossibly fast
    if (dt <= 0.0f) return 0.0f;

    // 2. Calculate Proportional Error
    float error = setpoint - measured_value;

    // 3. Calculate Integral with Anti-Windup
    integral += (error * dt);
    
    // Dynamic Anti-Windup: Prevent the integral from overwhelming the output limits
    // We clamp the integral based on Ki so (Ki * integral) doesn't exceed max output.
    if (ki > 0.0f) {
        float max_integral = output_max / ki;
        float min_integral = output_min / ki;
        if (integral > max_integral) integral = max_integral;
        if (integral < min_integral) integral = min_integral;
    }

    // 4. Calculate Derivative (Rate of change of error)
    float derivative = (error - previous_error) / dt;
    previous_error = error;

    // 5. Compute Final PID Output
    float output = (kp * error) + (ki * integral) + (kd * derivative);

    // 6. Clamp Output to Mechanical Limits
    if (output > output_max) output = output_max;
    if (output < output_min) output = output_min;

    return output;
}