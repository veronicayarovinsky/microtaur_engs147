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
#define I2C_TOF Wire

// // sensor objects
// VL53L4CD tof_left(&I2C_TOF, PIN_TOF_LEFT_GPIO);
// VL53L4CD tof_diag_left(&I2C_TOF, PIN_TOF_DIAG_L_GPIO);
// VL53L4CD tof_front(&I2C_TOF, PIN_TOF_FRONT_GPIO);
// VL53L4CD tof_diag_right(&I2C_TOF, PIN_TOF_DIAG_R_GPIO);
// VL53L4CD tof_right(&I2C_TOF, PIN_TOF_RIGHT_GPIO);

// uint8_t channels[NUM_TOF_SENSORS] = {TOF_CH_LEFT, TOF_CH_DIAG_L,
//                         TOF_CH_FRONT, TOF_CH_DIAG_R, TOF_CH_RIGHT};


// VL53L4CD tof_sensor_objects[NUM_TOF_SENSORS] = {
//     tof_left, tof_diag_left, tof_front, tof_diag_right, tof_right };


enum TofId {
    TOF_LEFT,
    TOF_DIAG_L,
    TOF_FRONT,
    TOF_DIAG_R,
    TOF_RIGHT,
    TOF_COUNT
};


void tof_init();

void tof_service();

// Last range_status for each sensor (0 = valid; see UM2931 for codes).
uint8_t tof_get_status(TofId id);

// non-blocking single-sensor attempt to read
// returns:
//      1 = new data read & stored
//      0 = sensor not ready yet (no blocking, try again later)
//      -1 = data ready but reading invalid
int  tof_try_read(TofId id);

// bool tof_read_if_ready();     // returns true when new data

// per-sensor freshness: true if new data has arrived since last consume.
bool tof_is_fresh(TofId id);
void tof_consume_fresh(TofId id);   // clear the fresh flag after using the data

// wall detection
void tof_check_walls_current_cell();
void tof_check_walls_next_cell();

#endif
