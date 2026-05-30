/**
 * @file tof_wall_detection_test.cpp
 * @brief print tof distance readings to display
 *        then print walls present (1, 0, or -1) current & next cell 
 */

#include <Arduino.h>
#include <ArduinoMotorShieldR3.h>
#include <AxisEncoderShield3.h>
#include <Wire.h>

#include "config.h"
#include "globals.h"
#include "tof.h"
#include "display.h"
#include "encoder.h"
#include "flood_fill.h"

#define S SerialUSB
#define NUM_SAMPLES 200

#define DT_TOF DT_CONTROL
#define TS_LED 1

ArduinoMotorShieldR3 md;

static unsigned long t_last_motors  = 0;
static unsigned long t_last_imu     = 0;
static unsigned long t_last_tof     = 0;
static unsigned long t_last_display = 0;
static unsigned long t_last_led     = 0;

static bool last_button_state = HIGH;

static const char* dir_name(Direction dir) {
    if (dir == NORTH) return "NORTH";
    if (dir == EAST)  return "EAST";
    if (dir == SOUTH) return "SOUTH";
    if (dir == WEST)  return "WEST";
    return "?";
}

void setup() {
    S.begin(BAUD_RATE);
    delay(200);

    pinMode(PIN_BUTTON, INPUT_PULLUP);

    display_init();
    md.init();
    encoder_init();
    tof_init();
    flood_init();

    unsigned long now = micros();

    t_last_motors  = now;
    t_last_imu     = now + 3000;
    t_last_tof     = now + 7000;
    t_last_display = now;

    delay(SETUP_DELAY_MS);

    S.println("starting");
    S.println("press button to calculate next flood fill direction");
}

void loop() {
    using namespace Micromouse;

    unsigned long times[NUM_SAMPLES];
    int16_t dL[NUM_SAMPLES], dDL[NUM_SAMPLES], dF[NUM_SAMPLES], dDR[NUM_SAMPLES], dR[NUM_SAMPLES];

    bool running = false;
    unsigned long sampleidx = 0;

    unsigned long now = micros();

    while (1) {
        now = micros();

        if (!running || now - t_last_tof >= T_TOF_US) {
            t_last_tof += T_TOF_US;

            encoder_update(DT_TOF);
            tof_service();

            running = true;
            t_last_tof = now;
            sampleidx++;
        }

        if (now - t_last_display >= T_DISPLAY_US) {
            t_last_display += T_DISPLAY_US;

            tof_check_walls_current_cell();
            tof_check_walls_next_cell();

            display_update();
        }

        bool button_state = digitalRead(PIN_BUTTON);

        if (last_button_state == HIGH && button_state == LOW) {
            delay(50);

            tof_check_walls_current_cell();
            tof_check_walls_next_cell();

            FloodOutput next_move = flood_fill_step(
                pose.x,
                pose.y,
                pose.a,
                walls_current_cell
            );

            display_direction(dir_name(next_move.want_dir));

            S.print("Current cell: ");
            S.print(pose.x);
            S.print(", ");
            S.println(pose.y);

            S.print("Move direction: ");
            S.println(dir_name(next_move.want_dir));

            S.print("Next cell: ");
            S.print(next_move.x_want);
            S.print(", ");
            S.println(next_move.y_want);

            // fake update for testing after you physically move it to the next cell
            pose.x = next_move.x_want;
            pose.y = next_move.y_want;
            pose.a = next_move.a_want;
        }

        last_button_state = button_state;
    }
}
