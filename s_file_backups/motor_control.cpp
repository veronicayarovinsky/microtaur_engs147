/**
 * @file motor_control.cpp
 * @brief compensator-based wheel speed controller. Reads encoder speeds and writes motor PWM commands.
 */

#include "motor_control.h"
#include "globals.h"
#include "config.h"

#include <Arduino.h>

// Motor direction signs
// From testing: forward required M1 negative and M2 positive.
// If the robot goes backward or spins, flip one of these signs.
static constexpr int LEFT_MOTOR_SIGN  = -1;
static constexpr int RIGHT_MOTOR_SIGN =  1;

// Compensator coefficients
// Difference equation:
// u[k] = V1*u[k-1] + E*e[k] - E1*e[k-1]
static constexpr float MOTOR_COMP_V1 = 1.0f;
static constexpr float MOTOR_COMP_E  = 0.68337f;
static constexpr float MOTOR_COMP_E1 = 0.5167f;

// Controller memory
static float s_left_output_prev  = 0.0f;
static float s_right_output_prev = 0.0f;

static float s_left_error_prev   = 0.0f;
static float s_right_error_prev  = 0.0f;

void motor_pi_init() {
    motor_pi_reset();
}

void motor_pi_reset() {
    s_left_output_prev  = 0.0f;
    s_right_output_prev = 0.0f;

    s_left_error_prev   = 0.0f;
    s_right_error_prev  = 0.0f;

    Micromouse::motors.pwm_left  = 0;
    Micromouse::motors.pwm_right = 0;
}

void motor_pi_update(float omega_left_ref, float omega_right_ref) {
    using namespace Micromouse;

    // calculate wheel speed errors
    float left_error  = omega_left_ref  - encoders.omega_left_rad_s;
    float right_error = omega_right_ref - encoders.omega_right_rad_s;

    // compensator difference equation
    // u[k] = V1*u[k-1] + E*e[k] - E1*e[k-1]
    float left_output = MOTOR_COMP_V1 * s_left_output_prev
                      + MOTOR_COMP_E  * left_error
                      - MOTOR_COMP_E1 * s_left_error_prev;

    float right_output = MOTOR_COMP_V1 * s_right_output_prev
                       + MOTOR_COMP_E  * right_error
                       - MOTOR_COMP_E1 * s_right_error_prev;

    // clamp PWM commands
    left_output  = constrain(left_output,  -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
    right_output = constrain(right_output, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);

    // save controller memory
    s_left_output_prev  = left_output;
    s_right_output_prev = right_output;

    s_left_error_prev   = left_error;
    s_right_error_prev  = right_error;

    // write final motor commands
    motors.pwm_left  = LEFT_MOTOR_SIGN  * (int)left_output;
    motors.pwm_right = RIGHT_MOTOR_SIGN * (int)right_output;
}
