/**
 * @file config.h
 * @brief all compile-time constants used by project code
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/* HARDCODE THESE BEFORE SOLVING MAZE*/
constexpr int X_START = 0;
constexpr int Y_START = 0;
constexpr int A_START = 0;

/* ***** Timing [sampling time "T_" in microseconds; time step "DT_" in seconds] ***** */
// T_ = sampling time; used for timing comparisons with micros()
// DT = differential time step used inside difference equations in controllers.cpp

constexpr unsigned long T_CONTROL_US = 10000;    // 10 ms (control loop sampling period)
constexpr float DT_CONTROL = T_CONTROL_US / 1e6f;   // dt = 0.01s (for math inside controllers)

constexpr unsigned long T_IMU_US     = 10000;   // 10 ms — IMU read
constexpr unsigned long T_TOF_US     = 15000;   // 15 ms — TOF read
// constexpr unsigned long T_DISPLAY_US = 250000;  // 250 ms — refresh display
constexpr unsigned long T_DISPLAY_US = 4000000;  // 4 s — refresh display
constexpr unsigned long SETUP_DELAY_MS = 200;

constexpr unsigned long TOF_RANGE_TIMING_BUDGET_MS = 40;  // tof sensor timing budget [ms]; smaller = faster but noisier

/* ***** Physical Measurements ***** */
// TODO: consider changing float to int (using multiplier)
constexpr float WHEEL_RADIUS_MM = 13.97f;  // 13.97 mm
constexpr float TRACK_WIDTH_MM  = 76.0f;    // distance between wheel centers
constexpr float CELL_SIZE_MM    = 180.0f;    // 18 cm maze cell
constexpr float WALL_HEIGHT_MM  = 0.050f;    // 5 cm maze wall height
constexpr float WALL_WIDTH_MM   = 0.012f;    // 1.2 cm maze wall width

/* ***** Encoder ***** */
constexpr float GEARBOX_REDUCTION_FACTOR = 100.37;      // from motor specs
constexpr float ENCODER_COUNTS_PER_REV = 12.0f * GEARBOX_REDUCTION_FACTOR;

#undef PI
constexpr float PI = 3.14159265f;
constexpr float WHEEL_CIRCUMFERENCE_MM = 2.0f * PI * WHEEL_RADIUS_MM;

constexpr float MM_PER_ENCODER_COUNT = WHEEL_CIRCUMFERENCE_MM / ENCODER_COUNTS_PER_REV;
constexpr float ENCODER_COUNTS_PER_MM = ENCODER_COUNTS_PER_REV / WHEEL_CIRCUMFERENCE_MM;
constexpr float ENCODER_COUNTS_PER_CELL = CELL_SIZE_MM * ENCODER_COUNTS_PER_MM;

/* ***** TOF thresholds ***** */
// TODO: test & adjust wall thresholds
// (wall-to-wall dist) - (robot width) = 16.8cm - 8cm = 8.8cm / 2 = 4.4cm
// but must account for worst-case scenario (robot against wall -> 8.8cm meas)
// and for 45deg tofs

constexpr float WALL_PRESENT_CURRENT_CELL_MM  = 88.0f;  // used by left, right, and front tofs for walls_current_cell
constexpr float WALL_PRESENT_NEXT_CELL_DIAG_MM  = 120.0f;  // used by left_diag and right_diag tofs for walls_next_cell
constexpr float WALL_PRESENT_NEXT_CELL_FRONT_MM  = 320.0f;  // used by left_diag and right_diag tofs for walls_next_cell
constexpr float TOF_MAX_RANGE_MM = 500.0f;  // 50 cm    TEST THIS 

/* ***** IMU thresholds ***** */
constexpr float HEADING_DONE_RAD = 0.035f;  // about 2 degrees
constexpr float OMEGA_DONE_RAD_S = 0.05f;

/* ***** Speed of Micromouse [m/s] ***** */
constexpr float SPEED_NOMINAL = 0.20f;
constexpr float SPEED_FAST    = 0.30f;

/* ***** Maze Map ***** */
constexpr unsigned short MAZE_SIZE = 16;

/* ***** Serial ***** */
constexpr unsigned long BAUD_RATE = 115200;

/* ***** Pins Arduino Due ***** */
// PCA9548 I2C mux
constexpr uint8_t I2C_ADDR_PCA9548  = 0x70;
constexpr uint8_t I2C_ADDR_BNO055   = 0x28;
constexpr uint8_t I2C_ADDR_VL53L4CD = 0x29;

constexpr int NUM_TOF_SENSORS = 5;
// CH refers to channel of I2C MUX that TOF sensor is connected to
constexpr int TOF_CH_LEFT = 3;
constexpr int TOF_CH_DIAG_L = 0;
constexpr int TOF_CH_FRONT = 7;
constexpr int TOF_CH_DIAG_R = 6;
constexpr int TOF_CH_RIGHT = 4;

// ; GPIO refers to pin on Arduino that TOF GPIO output is connected to
constexpr int PIN_TOF_LEFT_GPIO = 37;       // light blue
constexpr int PIN_TOF_DIAG_L_GPIO = 35;     // green
constexpr int PIN_TOF_FRONT_GPIO = 45;      // dark blue
constexpr int PIN_TOF_DIAG_R_GPIO = 43;     // grey
constexpr int PIN_TOF_RIGHT_GPIO = 39;      // purple



// Pushbutton
// active LOW aka LOW when pressed, HIGH when released
// example code for how to use:
/*  static bool last_btn = HIGH;
    bool btn = digitalRead(PIN_BUTTON);
    if (last_btn == HIGH && btn == LOW) {
        do_something();
        last_btn = btn;
    }                                        */
constexpr int PIN_BUTTON = 33;

// OLED display SPI
constexpr int PIN_OLED_CS  = 27;
constexpr int PIN_OLED_DC  = 29;
constexpr int PIN_OLED_RST = 31;

/* ***** I2C buses --> WIRE & WIRE1 */
// IMU on I2C0 (SDA=pin70, SCL=pin71) --> Wire (not marked on arduino due)
// TOF sensors on I2C1 (SDA=pin20, SCL=pin21) --> Wire1
//          (marked SCL1 & SDA1 on Arduino header; next to AREF pin)

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
