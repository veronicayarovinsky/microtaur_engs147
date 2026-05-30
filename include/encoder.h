/**
 * @file encoder.h
 * @brief writes --> Micromouse::encoders
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

void encoder_init();
void encoder_update(unsigned long actual_dt_us);  // pass actual elapsed time for correct velocity
void read_raw_counts(long& delta_l, long& delta_r);
void counts_to_distances(long delta_l, long delta_r, float& d_l_m, float& d_r_m);
void compute_kinematics(float d_l_m, float d_r_m, float dt_s);
void encoder_reset_odometry();

#endif
