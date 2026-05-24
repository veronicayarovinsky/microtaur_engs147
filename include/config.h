/**
 * @file config.h
 * @brief all compile-time constants used by project code
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/* ***** Timing [microseconds] ***** */
constexpr unsigned long T_CONTROL_US = 1000;    // 1 ms — motor PI + heading PD
constexpr unsigned long T_IMU_US     = 10000;   // 10 ms — IMU read
constexpr unsigned long T_TOF_US     = 15000;   // 15 ms — TOF read
constexpr unsigned long T_DISPLAY_US = 250000;  // 250 ms — refresh display

/* ***** Physical Measurements ***** */
constexpr float WHEEL_RADIUS_M = 0.01397f;  // 13.97 mm
constexpr float TRACK_WIDTH_M  = 0.080f;    // distance between wheel centers
constexpr float CELL_SIZE_M    = 0.180f;    // 18 cm maze cell
constexpr float WALL_HEIGHT_M  = 0.050f;    // 5 cm maze wall height
constexpr float WALL_WIDTH_M   = 0.012f;    // 1.2 cm maze wall width

/* ***** Encoder ***** */
constexpr float COUNTS_PER_REV = 1200.0f;

constexpr float WHEEL_CIRCUMFERENCE_M =
    2.0f * 3.14159265f * WHEEL_RADIUS_M;

constexpr float COUNTS_PER_METER =
    COUNTS_PER_REV / WHEEL_CIRCUMFERENCE_M;

constexpr float COUNTS_PER_CELL =
    CELL_SIZE_M * COUNTS_PER_METER;

/* ***** TOF thresholds ***** */
constexpr float WALL_PRESENT_M  = 0.12f;
constexpr float TOF_MAX_RANGE_M = 1.00f;

/* ***** IMU thresholds ***** */
constexpr float HEADING_DONE_RAD = 0.035f;  // about 2 degrees
constexpr float OMEGA_DONE_RAD_S = 0.05f;

/* ***** Speed of Micromouse [m/s] ***** */
constexpr float SPEED_NOMINAL = 0.20f;
constexpr float SPEED_FAST    = 0.30f;

/* ***** Motor PI Controller Coefficients ***** */
/*
 * PI controller form:
 *
 * u[k] = u[k-1] + B*e[k] + C*e[k-1]
 *
 * Backward-Euler style PI:
 * B = Kp + Ki*dt
 * C = -Kp
 */
constexpr float DT_MOTORS = T_CONTROL_US / 1000000.0f;

constexpr float KP_MOTOR = 1.0f;
constexpr float KI_MOTOR = 5.0f;

constexpr float MOTOR_PI_B = KP_MOTOR + KI_MOTOR * DT_MOTORS;
constexpr float MOTOR_PI_C = -KP_MOTOR;

constexpr float MOTOR_PWM_MAX = 400.0f;

/* ***** Heading PD Controller Coefficients ***** */
/*
 * PD controller form:
 *
 * u[k] = -u[k-1] + B*e[k] + C*e[k-1]
 *
 * Tustin derivative approximation:
 * B = Kp + 2*Kd/dt
 * C = Kp - 2*Kd/dt
 */
constexpr float DT_HEADING = T_IMU_US / 1000000.0f;

constexpr float KP_HEADING = 2.0f;
constexpr float KD_HEADING = 0.1f;

constexpr float HEADING_PD_B =
    KP_HEADING + 2.0f * KD_HEADING / DT_HEADING;

constexpr float HEADING_PD_C =
    KP_HEADING - 2.0f * KD_HEADING / DT_HEADING;

/* ***** Lateral PD Controller Coefficients ***** */
/*
 * Uses left/right TOF sensors to keep robot centered between walls.
 */
constexpr float DT_LATERAL = T_TOF_US / 1000000.0f;

constexpr float KP_LATERAL = 1.0f;
constexpr float KD_LATERAL = 0.05f;

constexpr float LATERAL_PD_B =
    KP_LATERAL + 2.0f * KD_LATERAL / DT_LATERAL;

constexpr float LATERAL_PD_C =
    KP_LATERAL - 2.0f * KD_LATERAL / DT_LATERAL;

/* ***** Maze Map ***** */
constexpr unsigned short MAZE_SIZE = 16;

/* ***** Serial ***** */
constexpr unsigned long BAUD_RATE = 115200;

/* ***** Pins Arduino Due ***** */

// Pushbutton
constexpr int PIN_BUTTON = 30;

// OLED display SPI
constexpr int PIN_OLED_CS  = 44;
constexpr int PIN_OLED_DC  = 46;
constexpr int PIN_OLED_RST = 48;

// PCA9548 I2C mux
constexpr uint8_t I2C_ADDR_PCA9548  = 0x70;
constexpr uint8_t I2C_ADDR_BNO055   = 0x28;
constexpr uint8_t I2C_ADDR_VL53L4CD = 0x29;

// I2C buses:
// TOF sensors on I2C0:
// SDA = pin 20
// SCL = pin 21
// Uses Wire
//
// IMU on I2C1:
// SDA = pin 70
// SCL = pin 71
// Uses Wire1

// Motor shield pins are defined in:
// ../lib/ArduinoMotorShieldR3/ArduinoMotorShieldR3.cpp
//
// DIR_A = 12
// PWM_A = 3
// CS_A  = A0
// DIR_B = 13
// PWM_B = 11
// CS_B  = A1

// Encoder shield pins are defined in:
// ../lib/AxisEncoderShield3/AxisEncoderShield.h
//
// CHIP_SEL_PIN_1 = 10
// CHIP_SEL_PIN_2 = 9
// CHIP_SEL_PIN_3 = 8

#endif
