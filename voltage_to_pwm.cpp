// #include "voltage_to_pwm.h"
#include <Arduino.h>

#define PWM_MAX     400
#define PWM_MIN     -400

const int n_points = 20;
const float voltage_control_points[n_points] = {
    -8.26, -8.15, -8.00, -7.82, -7.53, -7.08, -6.32, -5.50, -3.78, -0.18,
    0.16, 3.90, 5.52, 6.22, 7.16, 7.52, 7.80, 8.00, 8.15, 8.25
};
const int pwm_control_points[n_points] = {
    -400, -350, -300, -250, -200, -150, -100, -75, -50, -25,
    25, 50, 75, 100, 150, 200, 250, 300, 350, 400
};


long voltage_to_pwm(float v_input) {
    // check if voltage input is within valid range
    if (v_input <= voltage_control_points[0]) {
        return PWM_MIN;
    }
    if (v_input >= voltage_control_points[n_points - 1]) {
        return PWM_MAX;
    }

    // find interval (btw voltage control points) that contains input voltage value
    for (int i = 0; i < n_points - 1; i++) {
        if (v_input <= voltage_control_points[i + 1]) {
            // calc position of input voltage value relative to interval, as fraction 
            float fractional_pos = (v_input - voltage_control_points[i]) / (voltage_control_points[i + 1] - voltage_control_points[i]);
            
            // use fractional position of voltage interval to find corresponding position in pwm interval
            float pwm  = pwm_control_points[i] + fractional_pos * (pwm_control_points[i + 1] - pwm_control_points[i]);
            if (pwm >  PWM_MAX) {pwm =  PWM_MAX; }
            if (pwm < PWM_MIN) { pwm = PWM_MIN; }
            
            return (long)round(pwm);
        }
    }
    return 0;
}

