/**
 * @file main.cpp
 * @brief manages timing of control loops, sensor data readings, and fsm state transitions
 */

#include <Arduino.h>
#include <ArduinoMotorShieldR3.h>

#include "config.h"
#include "globals.h"
#include "display.h"
#include "encoder.h"
#include "imu.h"
#include "tof.h"
#include "drive.h"
#include "motor_control.h"
#include "heading_control.h"
#include "lateral_control.h"
#include "fsm.h"

// hardware objects
ArduinoMotorShieldR3 md;

// timing assumes that each loop iteration completes in less than the shortest timer interval

// global variables
static unsigned long t_last_control = 0;
static unsigned long t_last_imu = 0;
static unsigned long t_last_tof = 0;
static unsigned long t_last_display = 0;

void setup() {
    SerialUSB.begin(BAUD_RATE);

    // initialize hardware and software subsystems
    md.init();
    display_init();

    encoder_init();
    imu_init();
    tof_init();

    drive_init();
    fsm_init();

    // ensure all configuration is complete
    delay(SETUP_DELAY_MS);

    // initialize timing
    unsigned long current_time = micros();

    t_last_control = current_time;
    t_last_imu = current_time;
    t_last_tof = current_time;
    t_last_display = current_time;
}

void loop() {

    // loop-state variables
    unsigned long current_time = micros();

    // update motor control loop
    if (current_time - t_last_control >= T_CONTROL_US) {
        t_last_control += T_CONTROL_US;

        encoder_update();
        motor_pi_update();

        md.setM1Speed(motors.pwm_left);
        md.setM2Speed(motors.pwm_right);
    }

    // update imu loop
    if (current_time - t_last_imu >= T_IMU_US) {
        t_last_imu += T_IMU_US;

        imu_update();
        heading_pd_update();
    }

    // update tof loop
    if (current_time - t_last_tof >= T_TOF_US) {
        t_last_tof += T_TOF_US;

        tof_read_if_ready();
        lateral_pd_update();
    }

    // update fsm
    fsm_update();

    // update display
    if (current_time - t_last_display >= T_DISPLAY_US) {
        t_last_display += T_DISPLAY_US;
        display_update();   // ~5ms SPI transfer — safe here, never in 1ms block
    }
}
