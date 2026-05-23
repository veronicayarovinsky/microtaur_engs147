#include "motor_control.h"
#include "globals.h"
#include "config.h"
#include <Arduino.h>

// Left motor state
static float s_left_prev_error  = 0.0f;
static float s_left_prev_output = 0.0f;

// Right motor state
static float s_right_prev_error  = 0.0f;
static float s_right_prev_output = 0.0f;

void motor_pi_init()  { motor_pi_reset(); }

void motor_pi_reset() {
    s_left_prev_error   = 0.0f;
    s_left_prev_output  = 0.0f;
    s_right_prev_error  = 0.0f;
    s_right_prev_output = 0.0f;
    Micromouse::motors.pwm_left  = 0;
    Micromouse::motors.pwm_right = 0;
}

void motor_pi_update(float omega_left_ref, float omega_right_ref) {
    using namespace Micromouse;

    // Left motor
    float e_left         = omega_left_ref - encoders.omega_left_rad_s;
    float out_left       = s_left_prev_output
                         + MOTOR_PI_B * e_left
                         + MOTOR_PI_C * s_left_prev_error;
    out_left             = constrain(out_left, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
    s_left_prev_error    = e_left;
    s_left_prev_output   = out_left;
    motors.pwm_left      = (int)out_left;

    // Right motor
    float e_right        = omega_right_ref - encoders.omega_right_rad_s;
    float out_right      = s_right_prev_output
                         + MOTOR_PI_B * e_right
                         + MOTOR_PI_C * s_right_prev_error;
    out_right            = constrain(out_right, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
    s_right_prev_error   = e_right;
    s_right_prev_output  = out_right;
    motors.pwm_right     = (int)out_right;
}
