/**
 * @file imu.h
 * @brief BNO055 heading and angular velocity
 *        writes --> Micromouse::imu
 */

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

void imu_init();
void imu_update();
void get_imu_calibration_status();

#endif
