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

#define S SerialUSB
#define NUM_SAMPLES     200

#define DT_TOF      DT_CONTROL
#define TS_LED      1

ArduinoMotorShieldR3 md;
static unsigned long t_last_motors    = 0;
static unsigned long t_last_imu       = 0;
static unsigned long t_last_tof       = 0;
static unsigned long t_last_display   = 0;
static unsigned long t_last_led   = 0;


void setup() {
    S.begin(BAUD_RATE);
    delay(200);
    display_init();
    md.init();
    encoder_init();
    // imu_init();
    tof_init();
    
    unsigned long now     = micros();
    t_last_motors    = now;
    t_last_imu       = now + 3000;
    t_last_tof       = now + 7000;
    t_last_display   = now;
    
    delay(SETUP_DELAY_MS);

    S.println("starting");
}

void loop() {
    using namespace Micromouse;
    // one row per sample, columns = [t, dL, dDL, dF, dDR, dR]
    unsigned long times[NUM_SAMPLES];
    int16_t dL[NUM_SAMPLES], dDL[NUM_SAMPLES], dF[NUM_SAMPLES], dDR[NUM_SAMPLES], dR[NUM_SAMPLES];

    bool running = false;
    unsigned long sampleidx = 0;

    // ----- loop-state variables -----
    unsigned long prog_start, prev_loop_start, now;

    // ----- initialize timing -----
    now = micros();
    prog_start = now;
    prev_loop_start = now;

    while (1) {
        now = micros();
        
        if (!running || now - t_last_tof >= T_TOF_US) {
            t_last_tof += T_TOF_US;
            encoder_update(DT_TOF);
            tof_service();
            // update_controllers(v, omega);

            running = true;
            t_last_tof = now;
            sampleidx++;
        }
    

        // ----- check walls & update display -----
        if (now - t_last_display >= T_DISPLAY_US) {
            t_last_display += T_DISPLAY_US;
            tof_check_walls_current_cell();
            tof_check_walls_next_cell();
            display_update();   // ~5ms SPI transfer — safe here, never in 1ms block
        }

    }

}





