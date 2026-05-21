/**
 * @file config.h
 * @brief all compile-time constants used by project code
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/* ***** Timing [microseconds] ***** */
constexpr unsigned long T_CONTROL_US = 1000;   // 1 ms  — motor PI + heading PD
constexpr unsigned long T_IMU_US     = 10000;  // 10 ms — BNO055 read (100 Hz max in NDOF mode)
constexpr unsigned long T_TOF_US     = 15000;  // 15 ms — VL53L4CD timing budget 10ms + 4ms overhead + 1ms margin
constexpr unsigned long T_DISPLAY_US      = 250000; // 250ms — refresh display

/* ***** Encoder ***** */
constexpr float COUNTS_PER_REV  = 1200.0f;
constexpr float COUNTS_PER_CELL = (CELL_SIZE_M / (2.0f * 3.14159265f * WHEEL_RADIUS_M)) * COUNTS_PER_REV;

/* ***** TOF thresholds ***** */
constexpr float WALL_PRESENT_M   = 0.12f;
constexpr float TOF_MAX_RANGE_M  = 1.00f;

/* ***** IMU thresholds ***** */
constexpr float HEADING_DONE_RAD = 0.035f;  // ~2°
constexpr float OMEGA_DONE_RAD_S = 0.05f;

/* ***** Physical Measurements ***** */
constexpr float WHEEL_RADIUS_M  = 0.01397f;  // 13.97 mm (from BOM)
constexpr float TRACK_WIDTH_M   = 0.080f;    // L: distance between wheel centers
constexpr float CELL_SIZE_M     = 0.180f;    // 18 cm maze cell
constexpr float WALL_HEIGHT_M   = 0.050f;    // 5 cm maze wall height
constexpr float WALL_WIDTH_M    = 0.012f;    // 1.2 cm maze wall width

/* ***** Speed of Micromouse [m/s] ***** */
constexpr float SPEED_NOMINAL  = 0.20f;
constexpr float SPEED_FAST  = 0.30f;        // placeholder speed, if we get here

/* ***** Motor PI Controller Coefficients ***** */
constexpr float DT_MOTORS  = T_CONTROL_US / 1e6f;   // dt = 0.001s (for control loop rate)
constexpr float MOTOR_PWM_MAX = 400.0f;     // max; do not exceed
// +++++++++++++++++++++
// ADD CONTROLLER COEFFS
// +++++++++++++++++++++

/* ***** Heading PD Controller Coefficients ***** */
constexpr float DT_HEADING = T_IMU_US     / 1e6f;   // dt = 0.010s (for control loop rate)
// +++++++++++++++++++++
// ADD CONTROLLER COEFFS
// +++++++++++++++++++++

/* ***** Lateral PD Controller Coefficients ***** */
constexpr float DT_LATERAL = T_TOF_US     / 1e6f;   // dt = 0.015s (for control loop rate)
// +++++++++++++++++++++
// ADD CONTROLLER COEFFS
// +++++++++++++++++++++


/* ***** Maze Map ***** */
// +++++++++++++++++++++
// CHANGE & ADD
// +++++++++++++++++++++
constexpr unsigned short MAZE_SIZE = 16;    // 16×16 grid

/* ***** Serial ***** */
constexpr unsigned long BAUD_RATE = 115200;

/* ***** Pins (Arduino Due) ***** */
// pushbutton (active LOW aka LOW when pressed, HIGH when released)
constexpr int PIN_BUTTON = 30;

// OLED display (SPI — SCK & MOSI also used by encoder shield)
constexpr int PIN_OLED_CS = 44;
constexpr int PIN_OLED_DC = 46;
constexpr int PIN_OLED_RST = 48;

// PCA9548 I2C mux
constexpr uint8_t I2C_ADDR_PCA9548 = 0x70;
constexpr uint8_t I2C_ADDR_BNO055  = 0x28;  // ADR pulled low on Adafruit breakout board; (change to 0x29 if ADR high)
constexpr uint8_t I2C_ADDR_VL53L4CD = 0x29; // not used

// I2C buses
// TOF sensors on I2C0 (SDA=pin20, SCL=pin21) --> Wire
// IMU on I2C1 (SDA=pin70, SCL=pin71) --> Wire

// motor shield (defined in ../lib/ArduinoMotorShieldR3/ArduinoMotorShieldR3.cpp)
// DIR_A = 12;
// PWM_A = 3;
// CS_A = A0;
// DIR_B = 13;
// PWM_B = 11;
// CS_B = A1;

// encoder shield (defined in ../lib/AxisEncoderShield3/AxisEncoderShield.h)
// CHIP_SEL_PIN_1 10
// CHIP_SEL_PIN_2 9
// CHIP_SEL_PIN_3 8


#endif