/**
 * @file heading_control.cpp
 * @brief Heading PD controller and differential-drive mixer
 */

#include "heading_control.h"
#include "globals.h"
#include "config.h"

#include <Arduino.h>

// ----- controller memory -----
static float s_prev_error = 0.0f;
static float s_prev_output = 0.0f;

// ----- wheel speed references sent to motor PI -----
static float s_omega_left_ref = 0.0f;
static float s_omega_right_ref = 0.0f;

void heading_pd_init() {
    heading_pd_reset();
}

void heading_pd_reset() {
    s_prev_error = 0.0f;
    s_prev_output = 0.0f;

    s_omega_left_ref = 0.0f;
    s_omega_right_ref = 0.0f;
}

static float wrap_angle_rad(float angle_rad) {
    while (angle_rad > PI) {
        angle_rad -= 2.0f * PI;
    }

    while (angle_rad < -PI) {
        angle_rad += 2.0f * PI;
    }

    return angle_rad;
}

void heading_pd_update(float heading_ref_rad,
                       float v_base_m_s,
                       float lateral_correction_rad) {
    using namespace Micromouse;

    // ----- calculate heading error -----
    float desired_heading = heading_ref_rad + lateral_correction_rad;
    desired_heading = wrap_angle_rad(desired_heading);

    float error = desired_heading - imu.heading_rad;
    error = wrap_angle_rad(error);

    // ----- PD difference equation -----
    // output is desired robot angular velocity [rad/s]
    float omega_cmd = -s_prev_output
                    + HEADING_PD_B * error
                    + HEADING_PD_C * s_prev_error;

    // clamp so the robot does not try to rotate insanely fast
    omega_cmd = constrain(omega_cmd, -6.0f, 6.0f);

    // ----- save controller memory -----
    s_prev_error = error;
    s_prev_output = omega_cmd;

    // ----- differential drive kinematics -----
    // v_left  = v - omega * L/2
    // v_right = v + omega * L/2
    float v_left_m_s  = v_base_m_s - omega_cmd * TRACK_WIDTH_M / 2.0f;
    float v_right_m_s = v_base_m_s + omega_cmd * TRACK_WIDTH_M / 2.0f;

    // convert linear wheel speed [m/s] to angular wheel speed [rad/s]
    s_omega_left_ref = v_left_m_s / WHEEL_RADIUS_M;
    s_omega_right_ref = v_right_m_s / WHEEL_RADIUS_M;
}

float heading_pd_get_omega_left() {
    return s_omega_left_ref;
}

float heading_pd_get_omega_right() {
    return s_omega_right_ref;
}