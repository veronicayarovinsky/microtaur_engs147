/**
 * @file tof.h
 * @brief gets distance readings from 3x VL53L4CD TOF sensors thru PCA9548 mux
 *        & writes --> Micromouse::tof (distances and wall booleans)
 * 
 * Using polling mode
 * 
 * Datasheet: https://www.st.com/resource/en/datasheet/vl53l4cd.pdf
 * User Manual for API / functions: https://www.st.com/resource/en/user_manual/um2931-a-guide-to-using-the-vl53l4cd-ultra-lite-driver-uld-stmicroelectronics.pdf
 * Product Page: https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cd.html?ecmp=tt9470_gl_link_feb2019&rt=ds&id=DS13812#documentation
 * Arduino Lib Github: https://github.com/stm32duino/VL53L4CD
 */

#ifndef TOF_H
#define TOF_H

#include <Arduino.h>

void tof_init();
bool tof_read_if_ready();     // returns true when new data

#endif
