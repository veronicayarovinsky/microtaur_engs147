/**
 * @file main.cpp
 * @brief motor control timing test
 */

#include <Arduino.h>
#include <ArduinoMotorShieldR3.h>

#include "config.h"
#include "globals.h"
#include "encoder.h"
#include "motor_control.h"

using namespace Micromouse;

// hardware objects
ArduinoMotorShieldR3 md;

// test settings
constexpr float TEST_OMEGA_LEFT_RAD_S = 20.0f;
constexpr float TEST_OMEGA_RIGHT_RAD_S = 20.0f;

constexpr unsigned long TEST_RUN_TIME_US = 5000000UL;   // 5 seconds
constexpr unsigned long START_DELAY_MS = 3000;
constexpr unsigned long T_PRINT_US = 100000UL;          // 100 ms

// timing variables
static unsigned long t_last_control = 0;
static unsigned long t_last_print = 0;
static unsigned long t_start_run = 0;

static bool test_running = false;
static bool test_done = false;

void setup() {
    SerialUSB.begin(BAUD_RATE);

    delay(200);

    SerialUSB.println("Motor control test starting...");
    SerialUSB.println("Robot will wait 3 seconds before moving.");

    // initialize hardware
    md.init();
    encoder_init();
    motor_pi_init();

    // make sure motors are stopped
    md.setM1Speed(0);
    md.setM2Speed(0);

    delay(START_DELAY_MS);

    unsigned long current_time = micros();

    t_last_control = current_time;
    t_last_print = current_time;
    t_start_run = current_time;

    test_running = true;

    SerialUSB.println("Test running.");
}

void loop() {
    unsigned long current_time = micros();

    // stop after test time
    if (test_running && (current_time - t_start_run >= TEST_RUN_TIME_US)) {
        test_running = false;
        test_done = true;

        motor_pi_reset();

        md.setM1Speed(0);
        md.setM2Speed(0);

        SerialUSB.println("Test complete. Motors stopped.");
    }

    // 1 ms motor control loop
    if (test_running && (current_time - t_last_control >= T_CONTROL_US)) {
        unsigned long actual_dt_us = current_time - t_last_control;
        t_last_control += T_CONTROL_US;

        // update measured wheel speeds from encoders
        encoder_update(actual_dt_us);

        // run motor compensator / PI controller
        motor_pi_update(
            TEST_OMEGA_LEFT_RAD_S,
            TEST_OMEGA_RIGHT_RAD_S
        );

        // send PWM commands to motor shield
        md.setM1Speed(motors.pwm_left);
        md.setM2Speed(motors.pwm_right);
    }

    // print debug data
    if (current_time - t_last_print >= T_PRINT_US) {
        t_last_print += T_PRINT_US;

        SerialUSB.print("omega_L=");
        SerialUSB.print(encoders.omega_left_rad_s, 3);

        SerialUSB.print(", omega_R=");
        SerialUSB.print(encoders.omega_right_rad_s, 3);

        SerialUSB.print(", pwm_L=");
        SerialUSB.print(motors.pwm_left);

        SerialUSB.print(", pwm_R=");
        SerialUSB.println(motors.pwm_right);
    }

    // after test, keep motors paused
    if (test_done) {
        md.setM1Speed(0);
        md.setM2Speed(0);
    }
}
