#include "heading_control.h"
#include "globals.h"
#include "config.h"
#include <Arduino.h>

// Tustin (bilinear) PD difference equation for heading control:
//   v_k = -v_{k-1} + B*e_k + C*e_{k-1}
//
//     e_k    = heading_ref - heading_measured  (heading error at current tick)
//     e_{k-1} = heading error at previous tick
//     v_k    = omega_cmd (desired yaw rate [rad/s])
//     B = Kp + 2*Kd/T   (= HEADING_PD_B from config.h)
//     C = Kp - 2*Kd/T   (= HEADING_PD_C from config.h)


static float s_prev_error  = 0.0f;
static float s_prev_output = 0.0f;
static float s_omega_L_ref = 0.0f;
static float s_omega_R_ref = 0.0f;

void heading_pd_init()  { heading_pd_reset(); }

void heading_pd_reset() {
    s_prev_error  = 0.0f;
    s_prev_output = 0.0f;
    s_omega_L_ref = 0.0f;
    s_omega_R_ref = 0.0f;
}

void heading_pd_update(float heading_ref_rad,
                       float v_base_m_s,
                       float lateral_correction_rad) {
    using namespace Micromouse;

    if (!imu.ready) {
        s_omega_L_ref = 0.0f;
        s_omega_R_ref = 0.0f;
        return;
    }

    // Heading error — wrapped to [-π, π] to handle the 359°→1° boundary
    float err = (heading_ref_rad + lateral_correction_rad) - imu.heading_rad;
    while (err >  PI) err -= 2.0f * PI;
    while (err < -PI) err += 2.0f * PI;

    // Tustin PD difference equation
    float omega_cmd  = -s_prev_output
                     + HEADING_PD_B * err
                     + HEADING_PD_C * s_prev_error;

    s_prev_error  = err;
    s_prev_output = omega_cmd;

    // Kinematics mixer: (v_base, omega_cmd) → (omega_L, omega_R)
    float v_L     = v_base_m_s - omega_cmd * (TRACK_WIDTH_M / 2.0f);
    float v_R     = v_base_m_s + omega_cmd * (TRACK_WIDTH_M / 2.0f);
    s_omega_L_ref = v_L / WHEEL_RADIUS_M;
    s_omega_R_ref = v_R / WHEEL_RADIUS_M;
}

float heading_pd_get_omega_left()  { return s_omega_L_ref; }
float heading_pd_get_omega_right() { return s_omega_R_ref; }
