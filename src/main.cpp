/**
 * @file main.cpp
 * @brief 
 */

#include <Arduino.h>
#include <ArduinoMotorShieldR3.h>
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
    encoder_init();
    controllers_init();
    drive_init();

    delay(SETUP_DELAY_MS);
    S.println("Done initializing. Moving to ");
}

// outputs a change inputs 
// float choose_direction(int Aglobal, int Awant) {
//     Aturn = Aglobal - Awant;
//     if (Aturn = 1 
//     return Aturn
// } 

void loop() {
    using namespace Micromouse;
    // if x am I in cent
    if (7 <= pose.x <= 8 && 7 <= pose.x <= 8) {
        display_print();
        while (true);
    }

    
    tof_check_walls_current_cell();
    // floodfill reads walls_current_cell
    // floodfill outputs --> 


    

}