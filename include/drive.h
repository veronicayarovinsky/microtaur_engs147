/**
 * @file drive.h
 * @brief basic actions; movement through maze consists of a sequence of these actions, called by the FSM in main.
 */

#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>

/* initializes controllers */
void drive_init();

/* zeros motors & resets all controllers */
void drive_stop();

/* drives forward for a fixed distance */
bool drive_forward(float distance_mm, float speed_mm_s);

/* spins in place by delta_heading_rad (positive = left) */
bool drive_turn(float delta_heading_rad);

#endif
