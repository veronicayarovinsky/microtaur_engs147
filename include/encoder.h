/**
 * @file encoder.h
 * @brief writes --> Micromouse::encoders
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

void encoder_init();
void encoder_update(unsigned long actual_dt_us);  // pass actual elapsed time for correct velocity
void encoder_reset_odometry();

#endif
