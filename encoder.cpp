#include "encoder.h"
#include "globals.h"
#include "config.h"
#include <Arduino.h>
#include <AxisEncoderShield3.h>

static long s_left_prev  = 0;
static long s_right_prev = 0;

void encoder_init() { initEncoderShield(); }

void encoder_update(uint32_t actual_dt_us) {
    using namespace Micromouse;

    long raw_left  = getEncoderValue(1);  // verify channel vs physical wiring
    long raw_right = getEncoderValue(2);

    encoders.enc_left  = raw_left;
    encoders.enc_right = raw_right;

    float dt      = actual_dt_us / 1e6f;
    float d_left  = (float)(raw_left  - s_left_prev);
    float d_right = (float)(raw_right - s_right_prev);

    // Use actual dt (not nominal) for correct velocity — matches lab1 approach
    encoders.omega_left_rad_s  = (d_left  / COUNTS_PER_REV) * 2.0f * PI / dt;
    encoders.omega_right_rad_s = (d_right / COUNTS_PER_REV) * 2.0f * PI / dt;

    float d_m = ((d_left + d_right) / 2.0f / COUNTS_PER_REV) * 2.0f * PI * WHEEL_RADIUS_M;
    encoders.dist_traveled_m += fabsf(d_m);

    s_left_prev  = raw_left;
    s_right_prev = raw_right;
}

void encoder_reset_odometry() {
    s_left_prev  = getEncoderValue(1);
    s_right_prev = getEncoderValue(2);
    Micromouse::encoders.dist_traveled_m = 0.0f;
}
