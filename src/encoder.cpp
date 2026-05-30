/**
 * @file encoder.cpp
 * @brief reads encoder counts and calculates everything the controllers need
 *      
   fwd_change_m       — forward distance this sample [m]
   rot_change_rad     — rotation this sample [rad]
   omega_left_rad_s   — left wheel angular velocity [rad/s]
   omega_right_rad_s  — right wheel angular velocity [rad/s]
   dist_traveled_m    — accumulated forward distance [m]
   heading_rad        — accumulated rotation from encoders [rad]
 */

#include "encoder.h"
#include "globals.h"
#include "config.h"

#include <Arduino.h>
#include <AxisEncoderShield3.h>

static long s_left_prev  = 0;
static long s_right_prev = 0;


void read_raw_counts(long& delta_l, long& delta_r) {
    long raw_l = getEncoderValue(1);
    long raw_r = getEncoderValue(2);

    Micromouse::encoders.enc_left  = raw_l;
    Micromouse::encoders.enc_right = raw_r;

    delta_l = raw_l - s_left_prev;
    delta_r = raw_r - s_right_prev;

    s_left_prev  = raw_l;
    s_right_prev = raw_r;
}

// convert encoder count deltas --> wheel arc distances [mm]
void counts_to_distances(long delta_l, long delta_r, float& d_l_m, float& d_r_m) {
    d_l_m = (float)delta_l / ENCODER_COUNTS_PER_REV * 2.0f * PI * WHEEL_RADIUS_MM;
    d_r_m = (float)delta_r / ENCODER_COUNTS_PER_CELL * 2.0f * PI * WHEEL_RADIUS_MM;
}

void compute_kinematics(float d_l_m, float d_r_m, float dt_s) {
    using namespace Micromouse;

    // Per-sample changes (used by outer loops)
    encoders.fwd_change_mm   = (d_l_m + d_r_m) / 2.0f;
    encoders.rot_change_rad = (d_r_m - d_l_m) / TRACK_WIDTH_MM;

    // Wheel angular velocities (used by inner speed loop)
    encoders.omega_left_rad_s  = (d_l_m / WHEEL_RADIUS_MM) / dt_s;
    encoders.omega_right_rad_s = (d_r_m / WHEEL_RADIUS_MM) / dt_s;

    // Accumulated odometry
    encoders.dist_traveled_mm += fabsf(encoders.fwd_change_mm);
    encoders.heading_rad     += encoders.rot_change_rad;
}


void encoder_init() {
    initEncoderShield();

    // initialize previous counts so first velocity calculation is not huge
    s_left_prev  = getEncoderValue(1);
    s_right_prev = getEncoderValue(2);

    Micromouse::encoders.enc_left = s_left_prev;
    Micromouse::encoders.enc_right = s_right_prev;
    Micromouse::encoders.omega_left_rad_s = 0.0f;
    Micromouse::encoders.omega_right_rad_s = 0.0f;
    Micromouse::encoders.dist_traveled_mm = 0.0f;
}

void encoder_update(uint32_t actual_dt_us) {
    long delta_l, delta_r;
    read_raw_counts(delta_l, delta_r);

    float d_l_m, d_r_m;
    counts_to_distances(delta_l, delta_r, d_l_m, d_r_m);

    float dt_s = actual_dt_us / 1e6f;
    compute_kinematics(d_l_m, d_r_m, dt_s);
}

void encoder_reset_odometry() {
    s_left_prev  = getEncoderValue(1);
    s_right_prev = getEncoderValue(2);

    Micromouse::encoders.enc_left = s_left_prev;
    Micromouse::encoders.enc_right = s_right_prev;
    Micromouse::encoders.omega_left_rad_s = 0.0f;
    Micromouse::encoders.omega_right_rad_s = 0.0f;
    Micromouse::encoders.dist_traveled_mm = 0.0f;
    Micromouse::encoders.heading_rad = 0.0f;
}