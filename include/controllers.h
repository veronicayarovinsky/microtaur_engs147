// controllers.h
#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <Arduino.h>

void controllers_init();
void controllers_reset();

// velocity: target linear velocity (forward speed) [mm/s]
// omega: target angular velocity [rad/s] (positive = CCW)
void update_controllers(float velocity, float omega);

// for logging to serial
float controllers_get_fwd_error();    // forward position error [m]
float controllers_get_rot_error();    // rotation error [rad]
float controllers_get_omega_L_ref();  // left wheel speed setpoint [rad/s]
float controllers_get_omega_R_ref();  // right wheel speed setpoint [rad/s]

#endif