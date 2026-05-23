/**
 * @file tof.h
 * @brief gets distance readings from 3x VL53L4CD TOF sensors thru PCA9548 mux
 *        & writes --> Micromouse::tof (distances and wall booleans)
 */

#ifndef TOF_H
#define TOF_H

#include <Arduino.h>

void tof_init();
bool tof_read_if_ready();     // returns true when new data

#endif
