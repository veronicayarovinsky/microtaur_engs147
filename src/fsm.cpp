/**
 * @file fsm.cpp
 * @brief finite state machine for basic micromouse motion testing
 */

#include "fsm.h"
#include "globals.h"
#include "config.h"
#include "drive.h"

#include <Arduino.h>

using namespace Micromouse;

// FSM memory
static bool s_button_was_pressed = false;
static bool s_paused = false;
static bool s_motion_started = false;

// helper function for button press
static bool button_pressed_once() {
    bool button_is_pressed = (digitalRead(PIN_BUTTON) == LOW);

    if (button_is_pressed && !s_button_was_pressed) {
        s_button_was_pressed = true;
        return true;
    }

    if (!button_is_pressed) {
        s_button_was_pressed = false;
    }

    return false;
}

void fsm_init() {
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    state = State::INIT;

    gridcell.x = 0;
    gridcell.y = 0;

    walls.left = false;
    walls.right = false;
    walls.front = false;

    s_button_was_pressed = false;
    s_paused = false;
    s_motion_started = false;

    drive_stop();

    SerialUSB.println("FSM initialized.");
    SerialUSB.println("Robot will start automatically.");
    SerialUSB.println("Press button to pause/resume.");
}

void fsm_update() {

    // pause/resume button
    if (button_pressed_once()) {
        s_paused = !s_paused;

        if (s_paused) {
            SerialUSB.println("Paused.");
            drive_stop();
            state = State::PAUSED;
        } else {
            SerialUSB.println("Resumed.");
            state = State::AT_CELL;
        }
    }

    // if paused, keep robot stopped
    if (s_paused) {
        drive_stop();
        return;
    }

    // main FSM 
    switch (state) {

        // INIT: automatically begin first test motion
        case State::INIT:

            SerialUSB.println("Starting automatic run.");

            s_motion_started = false;
            state = State::DRIVING_FORWARD;

            break;

        // AT_CELL: robot is stopped at a cell center
        case State::AT_CELL:

            drive_stop();

            // Store current wall readings
            walls.left = tof.wall_left;
            walls.right = tof.wall_right;
            walls.front = tof.wall_front;

            SerialUSB.print("At cell x=");
            SerialUSB.print(gridcell.x);
            SerialUSB.print(" y=");
            SerialUSB.print(gridcell.y);
            SerialUSB.print(" | L=");
            SerialUSB.print(walls.left);
            SerialUSB.print(" F=");
            SerialUSB.print(walls.front);
            SerialUSB.print(" R=");
            SerialUSB.println(walls.right);

            // For now, keep driving forward one cell at a time.
            // Later this is where flood fill will choose the next move.
            s_motion_started = false;
            state = State::DRIVING_FORWARD;

            break;

        // DRIVING_FORWARD: move forward exactly one cell
        case State::DRIVING_FORWARD:

            if (!s_motion_started) {
                SerialUSB.println("Driving one cell.");
                s_motion_started = true;
            }

            if (drive_forward(CELL_SIZE_M, SPEED_NOMINAL)) {
                SerialUSB.println("Finished one cell.");

                drive_stop();

                // For now assume robot starts facing north/+y.
                // Later this should update based on current direction.
                gridcell.y += 1;

                s_motion_started = false;
                state = State::AT_CELL;
            }

            break;

        // TURNING: placeholder for turning tests later
        case State::TURNING:

            drive_stop();
            state = State::AT_CELL;

            break;

        // PAUSED: handled at top of function
        case State::PAUSED:

            drive_stop();

            break;

        // GOAL: stop at goal
        case State::GOAL:

            drive_stop();
            SerialUSB.println("Goal reached.");
            state = State::DONE;

            break;

        // DONE: final stopped state
        case State::DONE:

            drive_stop();

            break;

        // Safety default
        default:

            drive_stop();
            state = State::INIT;

            break;
    }
}