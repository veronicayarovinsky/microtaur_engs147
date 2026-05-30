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

// static unsigned long t_start = 0;
static unsigned long t_last_display   = 0;
// how much the robot needs to turn relatively
static int delta_turn = 0;
static Direction desired_direction = NORTH;



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

    // Init State

    if (state == State::INIT) {
        // not sure what else to put here but I'm assuming the init state is important for something
        state = State::AT_CELL; // Jump straight into your maze tracking loop
    }

    else if (state == State::AT_CELL) {
        // win check: am I in the center 4 cells of the maze?
        if (pose.x >= 7 && pose.x <= 8 && pose.y >= 7 && pose.y <= 8) {
           state = State::GOAL;
            return;
        }

            // front, left, and right sensors check if there's a wall there
            tof_check_walls_current_cell();

            // translate the relative directions of the sensors to global NESW
            Direction global_front = (Direction)((pose.a) % 4);
            Direction global_right = (Direction)((pose.a + 1) % 4);
            Direction global_left  = (Direction)((pose.a + 3) % 4);

            // true if there's a wall, false if there's no wall
            maze_set_wall(pose.x, pose.y, global_front, walls_current_cell.front);
            maze_set_wall(pose.x, pose.y, global_right, walls_current_cell.right);
            maze_set_wall(pose.x, pose.y, global_left,  walls_current_cell.left);

            // floodfill reads walls_current_cell and current position
            flood_fill();

            // floodfill outputs which cell it wants to move to 
            desired_direction = flood_get_best_direction(pose.x, pose.y);

            // calculate how much you have to turn represented as an integer
            delta_turn = pose.a - desired_direction;

            // avoid turning 270 degrees
            if (delta_turn > 2)  delta_turn -= 4;
            if (delta_turn < -2) delta_turn += 4;

            state = State::TURNING;

    }

    else if (state == State::TURNING) {
        // multiply change in direction (in integer form) by 90 degrees in radians
        if (drive_turn(delta_turn * (PI / 2.0f))) {
            pose.a = (int)desired_direction; // subtract because the directions go from 0 to 3 clockwise
            state = State::DRIVING_FORWARD;
        }
    }

    else if (state == State::DRIVING_FORWARD) {
        
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
    
    update_controllers(drive_command.v_base_mm_s, drive_command.heading_ref_rad);
    }