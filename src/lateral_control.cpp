#include "lateral_control.h"
#include "globals.h"
#include "config.h"
#include <Arduino.h>

static float s_prev_error   = 0.0f;
static float s_prev_output  = 0.0f;
static float s_correction   = 0.0f;
static bool  s_enabled      = false;

void lateral_pd_init()  { lateral_pd_reset(); }

void lateral_pd_reset() {
    s_prev_error  = 0.0f;
    s_prev_output = 0.0f;
    s_correction  = 0.0f;
}

void lateral_pd_set_enabled(bool enabled) {
    s_enabled = enabled;
    if (!enabled) {
        lateral_pd_reset();
    }
}

void lateral_pd_update() {
    using namespace Micromouse;

    // Do nothing if disabled or walls not present on both sides
    if (!s_enabled || !tof.wall_left || !tof.wall_right) {
        lateral_pd_reset();
        tof.data_refreshed = false;
        return;
    }

    // Lateral error: positive = robot too far right → correct left
    float err    = (tof.dist_left_m - tof.dist_right_m) / 2.0f;

    // Tustin PD difference equation
    float output = -s_prev_output
                 + LATERAL_PD_B * err
                 + LATERAL_PD_C * s_prev_error;

    output = constrain(output, -0.175f, 0.175f);  // clamp to ±10°

    s_prev_error  = err;
    s_prev_output = output;
    s_correction  = output;

    tof.data_refreshed = false;
}

float lateral_pd_get_correction() { return s_correction; }


