void heading_pd_init();
void heading_pd_reset();

// call after imu_update().
// reads Micromouse::imu, calculates omega refs thru kinematics mixer, and stores omega refs
void heading_pd_update(float heading_ref_rad,
                       float v_base_m_s,
                       float lateral_correction_rad = 0.0f);


float heading_pd_get_omega_left();
float heading_pd_get_omega_right();
