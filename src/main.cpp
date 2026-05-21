/**
 * @file main.cpp
 * @brief manages timing of control loops, sensor data readings, and fsm state transitions
 */

#include <Arduino.h>
#include <ArduinoMotorShieldR3.h>

#include "config.h"
#include "globals.h"
#include "display.h"

#define BAUD_RATE       115200
#define TS              5000UL              // loop period  [us]
#define TS_SEC          (TS / 1000000.0f)   // loop period [s]
#define DISPLAY_TS      250000UL            // update display every 250ms
#define SETUP_DELAY_MS  200


// timing assumes that each loop iteration completes in less than the shortest timer interval

// ----- global variables -----
static unsigned long t_last_display = 0;

void setup() {
    SerialUSB.begin(BAUD_RATE);
    display_init();

    // ensure all configuration is complete
    delay(SETUP_DELAY_MS);
}

void loop() {

    // ----- loop-state variables -----
    unsigned long prog_start, prev_loop_start, current_time;

    // ----- initialize timing -----
    current_time = micros();
    prog_start = current_time;
    prev_loop_start = current_time;

    

    // update display
    if (current_time - t_last_display >= DISPLAY_TS) {
        t_last_display += DISPLAY_TS;
        display_update();   // ~5ms SPI transfer — safe here, never in 1ms block
    }

}

