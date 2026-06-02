/**
 * @file main.cpp
 * @brief 
 */

#include <Arduino.h>
#include "config.h"
#include "globals.h"
#include "display.h"
#include "tof.h"
#include "imu.h"
#include "encoder.h"
#include <ArduinoMotorShieldR3.h>
#include <AxisEncoderShield3.h>
#include "controllers.h"
#include "drive.h"
#include "flood_fill.h"

// ArduinoMotorShieldR3 md;

ArduinoMotorShieldR3 md;


// #define TEST_RUN_TIME_US 2000000
#define DISPLAY_TS      250000UL            // update display every 250ms
#define SETUP_DELAY_MS  200
#define S SerialUSB
#define DT_TOF DT_CONTROL

// static unsigned long t_start = 0;
static unsigned long t_last_display   = 0;
static unsigned long t_last_tof     = 0;
// how much the robot needs to turn relatively
static int delta_turn = 0;
static Direction desired_direction = NORTH;

static const char* dir_name(Direction dir) {
    if (dir == NORTH) return "NORTH";
    if (dir == EAST)  return "EAST";
    if (dir == SOUTH) return "SOUTH";
    if (dir == WEST)  return "WEST";
    return "?";
}

void setup() {
    S.begin(BAUD_RATE);
    S.println("S1: initializing");
    delay(200);
    display_init();
    imu_init();
    delay(400);
    tof_init();
    encoder_init();
    md.init();
    controllers_init();
    drive_init();
    flood_init();

    while (!S) {}

    while (!Micromouse::imu.ready) {
        imu_update();
        delay(50);
    }

    delay(SETUP_DELAY_MS);
    S.println("Done initializing. Moving to ");
}

// find the change in angle necessary to go from direction direction to desired direction 
// float choose_direction(int A_global, int A_want) {
//     int A_delta_turn = A_global - A_want;
//     return A_delta_turn;
// } 

void loop() {
    using namespace Micromouse;
    unsigned long now = micros();

    // continuous sensor polling
    if (now - t_last_tof >= T_TOF_US) {
        t_last_tof = now;
        
        encoder_update(DT_TOF);
        tof_service(); 
    }

    // Init State

    if (state == State::INIT) {
        delay(1000);
        S.println("--- SETUP START ---");
        delay(1000);
        S.println("1");
        delay(1000);
        S.println("2");
        delay(1000);
        S.println("3");
        delay(1000);
        state = State::AT_CELL; // Jump straight into your maze tracking loop
    }

    else if (state == State::AT_CELL) {
        // win check: am I in the center 4 cells of the maze?
        if (pose.x >= 7 && pose.x <= 8 && pose.y >= 7 && pose.y <= 8) {
           state = State::GOAL;
        }
        else {
            // front, left, and right sensors check if there's a wall there
            tof_check_walls_current_cell();
            // S.print("left wall:"); S.println(tof.dist_left_mm);
            // S.print("right wall:"); S.println(tof.dist_right_mm);
            // S.print("front wall:"); S.println(tof.dist_front_mm);

            // flood fill logic
            FloodOutput next_move = flood_fill_step(
                pose.x,
                pose.y,
                pose.a,
                walls_current_cell
            );

            // display_direction(dir_name(next_move.want_dir));

            // S.print("Current cell: ");
            // S.print(pose.x);
            // S.print(", ");
            // S.println(pose.y);

            // S.print("Move direction: ");
            // S.println(dir_name(next_move.want_dir));

            // S.print("Next cell: ");
            // S.print(next_move.x_want);
            // S.print(", ");
            // S.println(next_move.y_want);

            // floodfill outputs which cell it wants to move to 
            desired_direction = next_move.want_dir;

            // calculate how much you have to turn represented as an integer
            delta_turn = pose.a - desired_direction;

            // avoid turning 270 degrees
            if (delta_turn > 2)  delta_turn -= 4;
            if (delta_turn < -2) delta_turn += 4;

            state = State::TURNING;
        }

    }

    else if (state == State::TURNING) {
        // multiply change in direction (in integer form) by 90 degrees in radians
        if (drive_turn(delta_turn * (PI / 2.0f))) {
            pose.a = (int)desired_direction;
            state = State::DRIVING_FORWARD;
        }
    }

    else if (state == State::DRIVING_FORWARD) {
        // front, left, and right sensors check distance from side walls to see how how far bot is from centerline
        tof_check_walls_current_cell();
        
        if (drive_forward(180.0f, 200.0f)) {
            
            // update global position
            if (pose.a == 0)      pose.y++; // Assuming 0 is NORTH
            else if (pose.a == 1) pose.x++; // Assuming 1 is EAST
            else if (pose.a == 2) pose.y--; // Assuming 2 is SOUTH
            else if (pose.a == 3) pose.x--; // Assuming 3 is WEST
            
            // cycle back to analyze the new cell we just entered
            state = State::AT_CELL;
        }
    }

    else if (state == State::GOAL) {
        drive_stop();
        display_print();
        state = State::DONE;
    }

    else if (state == State::DONE) {
        // Do nothing
    }
    
    if (state != State::TURNING) {
        update_controllers(drive_command.v_base_mm_s, drive_command.heading_ref_rad);
    }
    }