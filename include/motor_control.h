void motor_pi_init();
void motor_pi_reset();

// reads Micromouse::encoders; writes Micromouse::motors.
// omega_left_ref, omega_right_ref: target wheel speeds [rad/s].
void motor_pi_update(float omega_left_ref, float omega_right_ref);
