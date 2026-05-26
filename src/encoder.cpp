#include "encoder.h"
#include "globals.h"
#include "config.h"

#include <Arduino.h>
#include <AxisEncoderShield3.h>

static long s_left_prev  = 0;
static long s_right_prev = 0;

void encoder_init() {
    initEncoderShield();

    // initialize previous counts so first velocity calculation is not huge
    s_left_prev  = getEncoderValue(1);
    s_right_prev = getEncoderValue(2);

    Micromouse::encoders.enc_left = s_left_prev;
    Micromouse::encoders.enc_right = s_right_prev;
    Micromouse::encoders.omega_left_rad_s = 0.0f;
    Micromouse::encoders.omega_right_rad_s = 0.0f;
    Micromouse::encoders.dist_traveled_m = 0.0f;
}

void encoder_update(uint32_t actual_dt_us) {
    using namespace Micromouse;

    if (actual_dt_us == 0) {
        return;
    }

    long raw_left  = getEncoderValue(1);
    long raw_right = getEncoderValue(2);

    encoders.enc_left  = raw_left;
    encoders.enc_right = raw_right;

    float dt = actual_dt_us / 1000000.0f;

    float d_left_counts  = (float)(raw_left  - s_left_prev);
    float d_right_counts = (float)(raw_right - s_right_prev);

    // Left encoder sign was correct from testing:
    // left forward  -> positive omega
    float d_left_signed = d_left_counts;

    // Right encoder sign was flipped from testing:
    // right forward -> negative omega, so multiply by -1
    float d_right_signed = -d_right_counts;

    // wheel angular velocities [rad/s]
    encoders.omega_left_rad_s =
        (d_left_signed / COUNTS_PER_REV) * 2.0f * PI / dt;

    encoders.omega_right_rad_s =
        (d_right_signed / COUNTS_PER_REV) * 2.0f * PI / dt;

    // robot forward distance should use signed-forward wheel motion
    float d_m =
        ((d_left_signed + d_right_signed) / 2.0f / COUNTS_PER_REV)
        * 2.0f * PI * WHEEL_RADIUS_M;

    encoders.dist_traveled_m += fabsf(d_m);

    s_left_prev  = raw_left;
    s_right_prev = raw_right;
}

void encoder_reset_odometry() {
    s_left_prev  = getEncoderValue(1);
    s_right_prev = getEncoderValue(2);

    Micromouse::encoders.enc_left = s_left_prev;
    Micromouse::encoders.enc_right = s_right_prev;
    Micromouse::encoders.omega_left_rad_s = 0.0f;
    Micromouse::encoders.omega_right_rad_s = 0.0f;
    Micromouse::encoders.dist_traveled_m = 0.0f;
}