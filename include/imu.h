/**
 * @file imu.h
 * @brief BNO055 heading and angular velocity
 *        writes --> Micromouse::imu
 */

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "config.h"

void imu_init();
void imu_update();

#endif
