// controllers.cpp
#include "controllers.h"
#include "globals.h"
#include "config.h"
#include "imu.h"
#include <Arduino.h>
#include <ArduinoMotorShieldR3.h>
#include <math.h>

#define S SerialUSB
#define LOGGING 1


extern ArduinoMotorShieldR3 md;

// need these for turn control
#define TS_US 5000UL
#define RECORD_TIME_US 5000000UL


// ===== inner speed loop: PI =====
// Controls wheel speed. Reference = wheel omega [rad/s]. Output = volts.
// SPECS: fastest loop; bandwidth >= 3-5x the outer loops; 2% settling <= ~0.1 s;
//        overshoot <= ~10%; zeta >= 0.6; zero steady-state speed error (integrator).
// D(z) = (E - E1 z^-1)/(1 - z^-1) ;  u[k] = u[k-1] + E e[k] - E1 e[k-1]
constexpr float SPEED_V1 = 1.0f;
constexpr float SPEED_E  = 0.68337f;
constexpr float SPEED_E1 = 0.5167f;
constexpr float MOTOR_VOLT_MAX = 7.4f;

// ===== outer position loop: lead + integrator =====
// Setpoint-in: target distance [mm]; controls dist_traveled_mm. Output = body velocity [mm/s].
// SPECS: overshoot <= 5% (overshoot = past the stop point); zeta ~0.7;
//        2% settling ~0.3-0.6 s; omega_n ~8-15 rad/s; bandwidth <= ~1/3 of speed loop;
//        zero ss error to step AND ramp (type-2 -> tracks cruise velocity with no lag).
// D(z) = K z(z - z0)/[(z - 1)(z - p0)] ; 0 < p0 < z0 < 1
//   --> u[k] = (1+p0)u[k-1] - p0 u[k-2] + K e[k] - K z0 e[k-1]
constexpr float FWD_K  = 0.5f;    // loop gain  -- TODO
constexpr float FWD_Z0 = 0.90f;   // lead zero  -- TODO
constexpr float FWD_P0 = 0.40f;   // lead pole  -- TODO
constexpr float FWD_A1 = 1.0f + FWD_P0;
constexpr float FWD_A2 = -FWD_P0;
constexpr float FWD_B0 = FWD_K;
constexpr float FWD_B1 = -FWD_K * FWD_Z0;
constexpr float FWD_VEL_MAX = 400.0f;   // body-velocity clamp [mm/s] (anti-windup)

// ===== outer angle loop: lead + integrator =====
// Setpoint-in: target heading [rad]; controls imu heading. Output = body rate [rad/s].
// SPECS: overshoot <= 5%; zeta ~0.7-0.8; 2% settling ~0.3-0.5 s; omega_n ~11-19 rad/s
//        (target ~14); PM >= 45-60 deg; GM >= 6 dB; bandwidth <= ~1/3 of speed loop;
//        zero ss error to step AND ramp.
// D(z) = K z(z - z0)/[(z - 1)(z - p0)]
//   --> u[k] = (1+p0)u[k-1] - p0 u[k-2] + K e[k] - K z0 e[k-1]
// constexpr float ROT_K  = 12.0f;    // loop gain // 32.0f
// constexpr float ROT_Z0 = 0.95f;   // lead zero
// constexpr float ROT_P0 = 0.20f;   // lead pole
// constexpr float ROT_A1 = 1.0f + ROT_P0;
// constexpr float ROT_A2 = -ROT_P0;
// constexpr float ROT_B0 = ROT_K;
// constexpr float ROT_B1 = -ROT_K * ROT_Z0;
// constexpr float ROT_RATE_MAX = 4.0f;    // body-rate clamp [rad/s] (anti-windup)

// ===== outer angle loop: trapezoidal velocity profile + feedforward/P (heading only) =====
// Setpoint-in: final target heading [rad]; controls imu heading. Output = body rate [rad/s].
// The profile generates a moving heading reference (theta_ref) and its rate (omega_ref) --
// planned accel / cruise / decel -- so the reference itself decelerates ON SCHEDULE and
// arrives with zero velocity. Tracking = feedforward(omega_ref) + small P(theta_ref - heading).
// SPECS: no overshoot (deceleration is planned, not reactive); arrive within THETA_TOL;
//        OMEGA_MAX below physical max so the loop stays unsaturated/linear; heading-only.
constexpr float OMEGA_MAX = 3.0f;    // cruise body rate [rad/s] (below ~4.4 physical max)
constexpr float ALPHA_MAX = 4.0f;   // body angular accel [rad/s^2] (must be achievable)   // 12.0f
constexpr float ROT_KP    = 4.0f;    // heading-tracking trim [1/s] (FF does the work)
constexpr float THETA_TOL = 0.02f;   // ~1.1 deg arrival tolerance [rad]
constexpr float ROT_RATE_MAX = 4.0f; // safety clamp on body rate [rad/s]


// position state
static float s_fwd_e = 0.0f, s_fwd_e1 = 0.0f, s_fwd_u1 = 0.0f, s_fwd_u2 = 0.0f;

// angle / profile state
static float s_rot_e      = 0.0f;    // tracking error (exposed via getter)
static float s_theta_ref  = 0.0f;    // profile reference heading [rad]
static float s_omega_ref  = 0.0f;    // profile reference rate [rad/s]
static float s_rot_final  = 0.0f;    // commanded final heading [rad]
static bool  s_rot_active = false;

// speed state
static float s_uL = 0.0f, s_uR = 0.0f, s_eL = 0.0f, s_eR = 0.0f;

// reference wheel speeds (logging)
static float s_omega_L_ref = 0.0f, s_omega_R_ref = 0.0f;

//turn control tuning
// all of this was tuned in matlab :)
const float CTRL_B0 = 96.2709f;
const float CTRL_B1 = -188.7640f;
const float CTRL_B2 = 92.4932f;

const float CTRL_A1 = -1.7978f;
const float CTRL_A2 = 0.7978f;

const int PWM_MAX = 150;
const int PWM_STICTION = 95;

const float ANGLE_DEADBAND_RAD = 2.0f * PI / 180.0f;

float e0 = 0.0f, e1 = 0.0f, e2 = 0.0f;
float u0 = 0.0f, u1 = 0.0f, u2 = 0.0f;

static float wrap_pi(float a) {
    while (a > PI)  a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

int apply_pwm_limits(float u, float error_rad) {
    if (fabsf(error_rad) < ANGLE_DEADBAND_RAD) {
        return 0;
    }

    int pwm = (int)roundf(u);

    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;

    if (pwm > 0 && pwm < PWM_STICTION) pwm = PWM_STICTION;
    if (pwm < 0 && pwm > -PWM_STICTION) pwm = -PWM_STICTION;

    return pwm;
}

// turn function accepting radians
bool drive_turn(float target_rad) {
    // Convert radians input to degrees right away
    float target_deg = target_rad * 180.0f / PI;

    using namespace Micromouse;

    static unsigned long start = 0;
    static unsigned long prev = 0;
    static float heading0 = 0.0f;

    // initialization
    if (start == 0) {
        e0 = e1 = e2 = 0.0f;
        u0 = u1 = u2 = 0.0f;
        
        imu_update();
        heading0 = imu.heading_rad;
        
        start = micros();
        
        // prevent micros() from accidentally being exactly 0
        if (start == 0) start = 1; 
        
        prev = start;
        return false; // Not done turning
    }

    unsigned long now = micros();

    // has the record time passed?
    if (now - start >= RECORD_TIME_US) {
        md.setM1Speed(0);
        md.setM2Speed(0);
        
        // Reset start to 0 so the next time drive_turn is called, it initializes again
        start = 0;   
        return true; // Turn complete!
    }

    // control loop
    if (now - prev >= TS_US) {
        prev = now;

        imu_update();

        float heading_rel = wrap_pi(imu.heading_rad - heading0);
        float target_rel = target_deg * PI / 180.0f;
        
        e0 = wrap_pi(target_rel - heading_rel);

        u0 = CTRL_B0 * e0 + CTRL_B1 * e1 + CTRL_B2 * e2 - CTRL_A1 * u1 - CTRL_A2 * u2;

        int pwm_cmd = apply_pwm_limits(u0, e0);

        //  characterization showed +PWM,+PWM turns right/negative.
        // If the robot turns the wrong way, remove negative sign
        int motor_pwm = -pwm_cmd;

        md.setM1Speed(motor_pwm);
        md.setM2Speed(motor_pwm);

        float t = (now - start) / 1000000.0f;

        S.print(t, 4); S.print(",");
        S.print(target_deg, 2); S.print(","); 
        S.print(heading_rel * 180.0f / PI, 2); S.print(",");
        S.print(e0 * 180.0f / PI, 2); S.print(",");
        S.print(u0, 2); S.print(",");
        S.println(motor_pwm);

        e2 = e1;
        e1 = e0;

        u2 = u1;
        u1 = u0;
    }

    return false; // still turning, keep calling
}

// voltage -> pwm map (shield nonlinearity + stiction floor)
int voltage_to_pwm(float volt) {
    const float A = 2.507f, B = 0.384f, C = -68.2f;
    const float D = 0.9572f, E = -0.459f, F = 84.27f;
    const int PWM_STICTION = 70;   // TODO: MEASURE min pwm to start from rest, both dirs
    int pwm;
    if (volt < -0.001f) {
        pwm = (int)((-A * expf(-B * volt)) + C);
        if (pwm > -PWM_STICTION) pwm = -PWM_STICTION;
        if (pwm < -400) pwm = -400;
    } else if (volt > 0.001f) {
        pwm = (int)((D * expf(-E * volt)) + F);
        if (pwm < PWM_STICTION) pwm = PWM_STICTION;
        if (pwm > 400) pwm = 400;
    } else {
        pwm = 0;
    }
    return pwm;
}


// position_controller: setpoint-in lead+integrator. target [mm] -> body velocity [mm/s]
float position_controller(float target_mm) {
    using namespace Micromouse;
    float e = target_mm - encoders.dist_traveled_mm;
    s_fwd_e = e;
    float u = FWD_A1 * s_fwd_u1 + FWD_A2 * s_fwd_u2 + FWD_B0 * e + FWD_B1 * s_fwd_e1;
    u = constrain(u, -FWD_VEL_MAX, FWD_VEL_MAX);   // clamp; store clamped -> anti-windup
    s_fwd_u2 = s_fwd_u1; s_fwd_u1 = u; s_fwd_e1 = e;
    return u;
}

// // angle_controller: setpoint-in lead+integrator. target [rad] -> body rate [rad/s]
// float angle_controller(float target_rad) {
//     using namespace Micromouse;
//     float e = target_rad - imu_data.heading_rad;
//     while (e >  PI) e -= 2.0f * PI;
//     while (e < -PI) e += 2.0f * PI;
//     s_rot_e = e;
//     float u = ROT_A1 * s_rot_u1 + ROT_A2 * s_rot_u2 + ROT_B0 * e + ROT_B1 * s_rot_e1;
//     u = constrain(u, -ROT_RATE_MAX, ROT_RATE_MAX);
//     s_rot_u2 = s_rot_u1; s_rot_u1 = u; s_rot_e1 = e;
//     return u;
// }


// trapezoid: advance theta_ref / omega_ref one tick toward s_rot_final
void heading_profile_update() {
    float remaining = wrap_pi(s_rot_final - s_theta_ref);

    if (fabsf(remaining) < THETA_TOL && s_omega_ref == 0.0f) {
        s_theta_ref = s_rot_final;                  // arrived: hold reference at target
        return;
    }
    float stop_dist = (s_omega_ref * s_omega_ref) / (2.0f * ALPHA_MAX);
    float step = ALPHA_MAX * DT_CONTROL;

    if (fabsf(remaining) <= stop_dist) {            // start decel when remaining = stop dist
        s_omega_ref -= copysignf(step, s_omega_ref);
        if (fabsf(s_omega_ref) < step) s_omega_ref = 0.0f;
    } else {                                        // accelerate toward cruise
        s_omega_ref += copysignf(step, remaining);
        s_omega_ref = constrain(s_omega_ref, -OMEGA_MAX, OMEGA_MAX);
    }
    s_theta_ref += s_omega_ref * DT_CONTROL;
}

// angle_controller: trapezoid profile + velocity feedforward + P trim (heading only)
float angle_controller(float target_rad) {
    using namespace Micromouse;

    // (re)start the profile from the current heading whenever a new target is commanded
    if (!s_rot_active || target_rad != s_rot_final) {
        s_rot_final  = target_rad;
        s_theta_ref  = imu.heading_rad;
        s_omega_ref  = 0.0f;
        s_rot_active = true;
    }

    heading_profile_update();

    float e = wrap_pi(s_theta_ref - imu.heading_rad);
    s_rot_e = e;
    float u = s_omega_ref + ROT_KP * e;             // feedforward rate + P trim
    u = constrain(u, -ROT_RATE_MAX, ROT_RATE_MAX);
    return u;
}

// motor_speed_controller: per-wheel PI difference equation -> volts -> pwm
void motor_speed_controller(float wL_ref, float wR_ref) {
    using namespace Micromouse;
    float eL_prev = s_eL;
    s_eL = wL_ref - encoders.omega_left_rad_s;
    s_uL = SPEED_V1 * s_uL + SPEED_E * s_eL - SPEED_E1 * eL_prev;
    s_uL = constrain(s_uL, -MOTOR_VOLT_MAX, MOTOR_VOLT_MAX);

    float eR_prev = s_eR;
    s_eR = wR_ref - encoders.omega_right_rad_s;
    s_uR = SPEED_V1 * s_uR + SPEED_E * s_eR - SPEED_E1 * eR_prev;
    s_uR = constrain(s_uR, -MOTOR_VOLT_MAX, MOTOR_VOLT_MAX);

    int pwm_l = voltage_to_pwm(s_uL);
    int pwm_r = voltage_to_pwm(-s_uR);   // right motor mirrored
    motors.pwm_left = pwm_l; motors.pwm_right = pwm_r;
    md.setM1Speed(pwm_l);
    md.setM2Speed(pwm_r);
}

// update_controllers: setpoint-in. target_mm [mm], target_rad [rad]
void update_controllers(float target_mm, float target_rad) {
    // float fwd_output = position_controller(target_mm);   // mm/s
    float fwd_output = 0;
    float rot_output = angle_controller(target_rad);     // rad/s

    float v_L = fwd_output - rot_output * (TRACK_WIDTH_MM / 2.0f);   // mm/s
    float v_R = fwd_output + rot_output * (TRACK_WIDTH_MM / 2.0f);

    s_omega_L_ref = v_L / WHEEL_RADIUS_MM;
    s_omega_R_ref = v_R / WHEEL_RADIUS_MM;
    motor_speed_controller(s_omega_L_ref, s_omega_R_ref);
}

// inner-loop entry point for the speed test
void controllers_drive_wheel_speeds(float omega_L_ref, float omega_R_ref) {
    s_omega_L_ref = omega_L_ref;
    s_omega_R_ref = omega_R_ref;
    motor_speed_controller(omega_L_ref, omega_R_ref);
}

// command at the outer controller's output and log the controlled variable at the control rate
// for measuring open loop behavior of outer loops
void controllers_drive(float fwd_mm_s, float rot_rad_s) {
    float v_L = fwd_mm_s - rot_rad_s * (TRACK_WIDTH_MM / 2.0f);
    float v_R = fwd_mm_s + rot_rad_s * (TRACK_WIDTH_MM / 2.0f);
    s_omega_L_ref = v_L / WHEEL_RADIUS_MM;
    s_omega_R_ref = v_R / WHEEL_RADIUS_MM;
    motor_speed_controller(s_omega_L_ref, s_omega_R_ref);
}

void controllers_init() {
    controllers_reset();
    #if LOGGING
        // S.print("ROT_K="); S.println(ROT_K);
        // S.print("ROT_Z0="); S.println(ROT_Z0);
        // S.print("ROT_P0="); S.println(ROT_P0);
        // S.print("ROT_RATE_MAX="); S.println(ROT_RATE_MAX);
    #endif
}

void controllers_reset() {
    s_fwd_e = s_fwd_e1 = s_fwd_u1 = s_fwd_u2 = 0.0f;

    // s_rot_e = s_rot_e1 = s_rot_u1 = s_rot_u2 = 0.0f;
    s_rot_e = s_theta_ref = s_omega_ref = s_rot_final = 0.0f;
    s_rot_active = false;

    s_uL = s_uR = s_eL = s_eR = 0.0f;
    s_omega_L_ref = s_omega_R_ref = 0.0f;
    md.setM1Speed(0);
    md.setM2Speed(0);
}

float controllers_get_fwd_error()   { return s_fwd_e;   }
float controllers_get_rot_error()   { return s_rot_e;   }
float controllers_get_theta_ref()   { return s_theta_ref; }
float controllers_get_omega_ref()   { return s_omega_ref; }
float controllers_get_omega_L_ref() { return s_omega_L_ref; }
float controllers_get_omega_R_ref() { return s_omega_R_ref; }
