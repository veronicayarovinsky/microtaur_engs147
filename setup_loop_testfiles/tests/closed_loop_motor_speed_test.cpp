/**
 * @file main.cpp
 * @brief closed-loop motor speed test
 */

#include <Arduino.h>
#include <ArduinoMotorShieldR3.h>

#include "config.h"
#include "globals.h"
#include "encoder.h"
#include "motor_control.h"

using namespace Micromouse;

ArduinoMotorShieldR3 md;

// ----- test settings -----
constexpr float TEST_OMEGA_LEFT_RAD_S  = 12.0f;
constexpr float TEST_OMEGA_RIGHT_RAD_S = 12.0f;

constexpr unsigned long START_DELAY_MS = 3000;
constexpr unsigned long TEST_RUN_TIME_US = 5000000UL;  // 5 seconds
constexpr unsigned long T_PRINT_US = 100000UL;         // 100 ms

// ----- timing variables -----
static unsigned long t_last_control = 0;
static unsigned long t_last_print = 0;
static unsigned long t_start = 0;

static bool test_running = false;
static bool test_done = false;
static bool stop_message_printed = false;

void stop_motors() {
    motor_pi_reset();

    md.setM1Speed(0);
    md.setM2Speed(0);

    motors.pwm_left = 0;
    motors.pwm_right = 0;
}

void setup() {
    Serial.begin(115200);

    delay(200);

    Serial.println("Closed-loop motor speed test starting...");
    Serial.println("Put robot on blocks with wheels off the ground.");
    Serial.println("Starting in 3 seconds...");

    md.init();
    encoder_init();
    motor_pi_init();

    stop_motors();

    delay(START_DELAY_MS);

    unsigned long current_time = micros();

    t_last_control = current_time;
    t_last_print = current_time;
    t_start = current_time;

    test_running = true;
    test_done = false;
    stop_message_printed = false;

    Serial.println("time_s,omega_L_ref,omega_L,omega_R_ref,omega_R,pwm_L,pwm_R");
}

void loop() {
    unsigned long current_time = micros();

    // stop after test time
    if (test_running && current_time - t_start >= TEST_RUN_TIME_US) {
        test_running = false;
        test_done = true;

        stop_motors();

        if (!stop_message_printed) {
            Serial.println("Test complete. Motors stopped.");
            stop_message_printed = true;
        }
    }

    // ----- 1 ms encoder/control loop -----
    if (current_time - t_last_control >= T_CONTROL_US) {
        unsigned long actual_dt_us = current_time - t_last_control;
        t_last_control += T_CONTROL_US;

        // Always update encoders so speed goes to zero after stopping
        encoder_update(actual_dt_us);

        // Only run motor controller while test is active
        if (test_running) {
            motor_pi_update(
                TEST_OMEGA_LEFT_RAD_S,
                TEST_OMEGA_RIGHT_RAD_S
            );

            md.setM1Speed(motors.pwm_left);
            md.setM2Speed(motors.pwm_right);
        } else {
            md.setM1Speed(0);
            md.setM2Speed(0);
        }
    }

    // ----- print data every 100 ms -----
    if (current_time - t_last_print >= T_PRINT_US) {
        t_last_print += T_PRINT_US;

        float time_s = (current_time - t_start) / 1000000.0f;

        Serial.print(time_s, 3);
        Serial.print(",");

        Serial.print(TEST_OMEGA_LEFT_RAD_S, 3);
        Serial.print(",");
        Serial.print(encoders.omega_left_rad_s, 3);
        Serial.print(",");

        Serial.print(TEST_OMEGA_RIGHT_RAD_S, 3);
        Serial.print(",");
        Serial.print(encoders.omega_right_rad_s, 3);
        Serial.print(",");

        Serial.print(motors.pwm_left);
        Serial.print(",");
        Serial.println(motors.pwm_right);
    }
}
