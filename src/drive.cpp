#include <Arduino.h>
#include "config.h"
#include "globals.h"
#include "encoder.h"
#include "controllers.h"
#include "drive.h"

// drive.cpp
// step 1: turn(Aturn)
// step 2: drive forward 1 cell

static bool  s_fwd_active      = false;
static float s_fwd_heading_ref = 0.0f;
static bool  s_turn_active     = false;
static float s_turn_target     = 0.0f;

void drive_init() {
    controllers_init();
    s_fwd_active  = false;
    s_turn_active = false;
}

void drive_stop() {
    controllers_reset();
    encoder_reset_odometry();
    Micromouse::drive_command.v_base_mm_s      = 0.0f;
    Micromouse::drive_command.heading_ref_rad = Micromouse::imu.heading_rad;
    Micromouse::motors.pwm_left  = 0;
    Micromouse::motors.pwm_right = 0;
    s_fwd_active  = false;
    s_turn_active = false;
}

bool drive_forward(float distance_mm, float speed_mm_s) {
    using namespace Micromouse;

    if (!s_fwd_active) {
        if (!imu.ready) return false;          // wait for IMU before locking heading
        s_fwd_active      = true;
        s_fwd_heading_ref = imu.heading_rad;   // lock heading at moment of first call
        s_turn_active     = false;
        encoder_reset_odometry();
    }

    // write drive command to globals
    drive_command.heading_ref_rad = s_fwd_heading_ref;
    drive_command.v_base_mm_s      = speed_mm_s;

    if (encoders.dist_traveled_mm >= distance_mm) {
        drive_stop();
        return true;
    }
    return false;
}

static bool heading_settled() {
    using namespace Micromouse;
    float err = s_turn_target - imu.heading_rad;
    while (err >  PI) err -= 2.0f * PI;
    while (err < -PI) err += 2.0f * PI;
    return fabsf(err) < HEADING_DONE_RAD
        && fabsf(imu.omega_imu_rad_s) < OMEGA_DONE_RAD_S;
}

bool drive_turn(float delta_heading_rad) {
    using namespace Micromouse;

    if (!s_turn_active) {
        if (!imu.ready) return false;
        s_turn_active = true;
        s_fwd_active  = false;
        s_turn_target = imu.heading_rad + delta_heading_rad; // change this to include centerline error
        while (s_turn_target >  PI) s_turn_target -= 2.0f * PI;
        while (s_turn_target < -PI) s_turn_target += 2.0f * PI;
    }

    drive_command.heading_ref_rad = s_turn_target;
    drive_command.v_base_mm_s      = 0.0f;            // pure rotation

    if (heading_settled()) {
        drive_stop();
        return true;
    }
    return false;
}
