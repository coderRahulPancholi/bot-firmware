#pragma once

#include <stdint.h>

class PIDController {
private:
    // Tuning Parameters
    float kp;
    float ki;
    float kd;

    // Output Limits
    float output_min;
    float output_max;

    // Memory Variables for Math
    float integral;
    float previous_error;
    int64_t last_time_us;

    // Feature Flags
    bool first_run;

public:
    // Constructor
    PIDController(float p, float i, float d, float min_out, float max_out);

    // The core calculation. Returns the calculated steering effort.
    float compute(float setpoint, float measured_value);

    // Update tunings on the fly (useful for Bluetooth/WiFi tuning apps)
    void setTunings(float p, float i, float d);

    // Reset the internal memory (Call this if the robot is picked up or reset)
    void reset();
};