// controllers.cpp

#include "controllers.h"
#include "globals.h"
#include "config.h"
#include <Arduino.h>
#include <ArduinoMotorShieldR3.h>

extern ArduinoMotorShieldR3 md;

// --------------------------
// outer position loop
constexpr float FWD_KP = 2.0f;    // (m/s) / m = s⁻¹
constexpr float FWD_KD = 1.1f;

// outer angle loop
constexpr float ROT_KP = 2.1f;    // (rad/s) / rad = s⁻¹
constexpr float ROT_KD = 1.2f;

// inner motor speed loop
constexpr float SPEED_KP         = 1.0f;   // PWM / (rad/s)
constexpr float SPEED_KI         = 5.0f;   // PWM / (rad/s × s)
constexpr float MOTOR_INTEGRAL_MAX = 200.0f;

constexpr float MOTOR_MAX_PWM = 400;
// --------------------------



// Position controller (forward)
static float s_fwd_error      = 0.0f;
static float s_prev_fwd_error = 0.0f;

// Angle controller (rotation)
static float s_rot_error      = 0.0f;
static float s_prev_rot_error = 0.0f;
// VERSION B only: tracks the commanded heading as it advances each tick
static float s_rot_target     = 0.0f;

// Motor speed controller (inner PI, one per wheel)
static float s_integral_L     = 0.0f;
static float s_integral_R     = 0.0f;

// Stored wheel speed references (for logging)
static float s_omega_L_ref    = 0.0f;
static float s_omega_R_ref    = 0.0f;

// position_controller
// every time fn is called, setpoint position increases by velocity × dt
// actual pos increases by what encoders measure
// error = (where micromouse should be) - (where micromouse is)
// returns: forward speed correction [m/s]
static float position_controller(float velocity) {
    using namespace Micromouse;

    s_fwd_error      = s_fwd_error + (velocity * DT_MOTORS) - encoders.fwd_change_mm;
    s_prev_fwd_error = s_fwd_error;

    return FWD_KP * s_fwd_error + FWD_KD * (s_fwd_error - s_prev_fwd_error);
}

// angle_controller
// returns: rotation correction [rad/s]
static float angle_controller(float omega) {
    using namespace Micromouse;

    // encoder-based
    s_rot_error += s_rot_error + (omega * DT_MOTORS) - encoders.rot_change_rad;

    // IMU-based
    // s_rot_target   = s_rot_target + (omega * DT_MOTORS);  // increment commanded heading
    // float imu_err  = s_rot_target - imu.heading_rad;
    // while (imu_err >  PI) {
    //     imu_err = imu_err - 2.0f * PI;  // wrap to [-π, π]
    // }
    // while (imu_err < -PI) {
    //     imu_err = imu_err + 2.0f * PI;
    // }
    // s_rot_error    = imu_err;  // not accumulated — recomputed each tick from absolute heading

    float diff       = s_rot_error - s_prev_rot_error;
    s_prev_rot_error = s_rot_error;

    return ROT_KP * s_rot_error + ROT_KD * diff;
}

// motor_speed_controller
// Anti-windup: integrator is clamped to MOTOR_INTEGRAL_MAX.
static void motor_speed_controller(float omega_L_ref, float omega_R_ref) {
    using namespace Micromouse;

    // Left wheel
    float err_L    = omega_L_ref - encoders.omega_left_rad_s;
    s_integral_L  += err_L * DT_MOTORS;
    s_integral_L   = constrain(s_integral_L, -MOTOR_INTEGRAL_MAX, MOTOR_INTEGRAL_MAX);
    float out_L    = SPEED_KP * err_L + SPEED_KI * s_integral_L;

    // Right wheel
    float err_R    = omega_R_ref - encoders.omega_right_rad_s;
    s_integral_R  += err_R * DT_MOTORS;
    s_integral_R   = constrain(s_integral_R, -MOTOR_INTEGRAL_MAX, MOTOR_INTEGRAL_MAX);
    float out_R    = SPEED_KP * err_R + SPEED_KI * s_integral_R;

    // Apply PWM
    int pwm_l = constrain((int)out_L, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
    int pwm_r = constrain((int)out_R, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
    motors.pwm_left  = pwm_l;
    motors.pwm_right = pwm_r;
    md.setM1Speed(pwm_l);
    md.setM2Speed(pwm_r);
}

// update_controllers
void update_controllers(float velocity, float omega) {
    float fwd_output = position_controller(velocity);
    float rot_output = angle_controller(omega);

    // kinematics mixer for differential drive
    float v_L = fwd_output - rot_output * (TRACK_WIDTH_M / 2.0f);
    float v_R = fwd_output + rot_output * (TRACK_WIDTH_M / 2.0f);

    s_omega_L_ref = v_L / WHEEL_RADIUS_M;
    s_omega_R_ref = v_R / WHEEL_RADIUS_M;

    motor_speed_controller(s_omega_L_ref, s_omega_R_ref);
}


void controllers_init()  {
    controllers_reset();
}

void controllers_reset() {
    s_fwd_error      = 0.0f;
    s_prev_fwd_error = 0.0f;

    s_rot_error      = 0.0f;
    s_prev_rot_error = 0.0f;
    s_rot_target     = 0.0f;

    s_integral_L     = 0.0f;
    s_integral_R     = 0.0f;

    s_omega_L_ref    = 0.0f;
    s_omega_R_ref    = 0.0f;
    md.setM1Speed(0);
    md.setM2Speed(0);
}


float controllers_get_fwd_error()   { return s_fwd_error;   }
float controllers_get_rot_error()   { return s_rot_error;   }
float controllers_get_omega_L_ref() { return s_omega_L_ref; }
float controllers_get_omega_R_ref() { return s_omega_R_ref; }
