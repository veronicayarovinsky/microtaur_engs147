// controllers.h
#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <Arduino.h>

void controllers_init();
void controllers_reset();

// voltage -> pwm map (motor-shield nonlinearity, command-sign frame)
int voltage_to_pwm(float volt);

// position_controller: returns forward speed correction [mm/s]
float position_controller(float velocity);



// angle_controller (IMU-based): returns rotation correction [rad/s]
void heading_profile_update();
float angle_controller(float omega);

// motor_speed_controller: per-wheel difference equation -> volts -> pwm
void motor_speed_controller(float wL_ref, float wR_ref);

// velocity: target linear velocity (forward speed) [mm/s]
// omega: target angular velocity [rad/s] (positive = CCW)
void update_controllers(float velocity, float omega);

// inner-loop entry point for the motor speed step response test
void controllers_drive_wheel_speeds(float omega_L_ref, float omega_R_ref);

void controllers_drive(float fwd_mm_s, float rot_rad_s);

// for logging to serial
float controllers_get_fwd_error();    // forward position error [m]
float controllers_get_rot_error();    // rotation error [rad]
float controllers_get_theta_ref();
float controllers_get_omega_ref();
float controllers_get_omega_L_ref();  // left wheel speed setpoint [rad/s]
float controllers_get_omega_R_ref();  // right wheel speed setpoint [rad/s]

#endif