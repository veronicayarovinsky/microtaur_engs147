/**
 * @file tof_wall_test_with_serialout.cpp
 * @brief Logs tof distances, wall flags, and function durations to arrays.
 *        Updates the OLED live during the run (only when it won't disturb
 *        TOF sample timing). Prints CSV at the end.
 *        Raw readings logged (not averaged) — decide on averaging from data.
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
#define NUM_SAMPLES   200
#define DT_TOF        DT_CONTROL

#define DISPLAY_GUARD_US   6000  // display_update() is a ~5ms SPI transfer; skip display if < this much time before next sample

// Log arrays
static unsigned long times[NUM_SAMPLES];
static int16_t  dL[NUM_SAMPLES], dDL[NUM_SAMPLES], dF[NUM_SAMPLES];
static int16_t  dDR[NUM_SAMPLES], dR[NUM_SAMPLES];
static uint8_t  walls[NUM_SAMPLES];          // packed wall flags
static uint16_t dur_service[NUM_SAMPLES];    // tof_service() duration [us]
static uint16_t dur_walls[NUM_SAMPLES];      // wall-check duration [us]

ArduinoMotorShieldR3 md;

// =============================================================================
// Helpers
// =============================================================================

// Pack the 6 wall flags into one byte for compact storage.
static uint8_t pack_walls() {
    using namespace Micromouse;
    uint8_t w = 0;
    w |= (walls_current_cell.left  ? 1 : 0) << 0;
    w |= (walls_current_cell.right ? 1 : 0) << 1;
    w |= (walls_current_cell.front ? 1 : 0) << 2;
    w |= (walls_next_cell.left     ? 1 : 0) << 3;
    w |= (walls_next_cell.right    ? 1 : 0) << 4;
    w |= (walls_next_cell.front    ? 1 : 0) << 5;
    return w;
}

// One timed sample: read sensors, check walls, time it, store it.
static void take_sample(int i, unsigned long t_rel) {
    using namespace Micromouse;

    encoder_update(DT_TOF);

    unsigned long t0 = micros();
    tof_service();
    unsigned long t1 = micros();

    tof_check_walls_current_cell();
    tof_check_walls_next_cell();
    unsigned long t2 = micros();

    times[i]       = t_rel;
    dL[i]          = tof.dist_left_mm;
    dDL[i]         = tof.dist_diag_left_mm;
    dF[i]          = tof.dist_front_mm;
    dDR[i]         = tof.dist_diag_right_mm;
    dR[i]          = tof.dist_right_mm;
    walls[i]       = pack_walls();
    dur_service[i] = (uint16_t)(t1 - t0);
    dur_walls[i]   = (uint16_t)(t2 - t1);
}

// Collect NUM_SAMPLES at the TOF rate. Update the OLED at the display rate,
// but only when doing so won't overrun into the next TOF sample.
static void collect_samples() {
    bool running = false;
    int  idx     = 0;
    unsigned long prog_start  = micros();
    unsigned long t_last_tof  = prog_start;
    unsigned long t_last_disp = prog_start;

    while (idx < NUM_SAMPLES) {
        unsigned long now = micros();

        // TOF rate: take and store a sample (timing-critical — always wins)
        if (!running || (now - t_last_tof >= T_TOF_US)) {
            t_last_tof = now;
            running = true;
            take_sample(idx, now - prog_start);
            idx++;
        }

        // Display rate: only update if a TOF sample is NOT about to fire and
        // there's enough slack to finish the ~5ms transfer in time.
        else if (now - t_last_disp >= T_DISPLAY_US) {
            unsigned long until_next_tof = T_TOF_US - (now - t_last_tof);
            if (until_next_tof > DISPLAY_GUARD_US) {
                t_last_disp = now;
                display_update();
            }
            // else: too close to next sample — skip this refresh, retry later
        }
    }
}

// Print all logged samples as CSV.
static void print_samples() {
    S.println("idx,t_us,dL_mm,dDL_mm,dF_mm,dDR_mm,dR_mm,"
              "curL,curR,curF,nextL,nextR,nextF,service_us,walls_us");
    for (int i = 0; i < NUM_SAMPLES; i++) {
        S.print(i);                   S.print(",");
        S.print(times[i]);            S.print(",");
        S.print(dL[i]);               S.print(",");
        S.print(dDL[i]);              S.print(",");
        S.print(dF[i]);               S.print(",");
        S.print(dDR[i]);              S.print(",");
        S.print(dR[i]);               S.print(",");
        S.print((walls[i] >> 0) & 1); S.print(",");
        S.print((walls[i] >> 1) & 1); S.print(",");
        S.print((walls[i] >> 2) & 1); S.print(",");
        S.print((walls[i] >> 3) & 1); S.print(",");
        S.print((walls[i] >> 4) & 1); S.print(",");
        S.print((walls[i] >> 5) & 1); S.print(",");
        S.print(dur_service[i]);      S.print(",");
        S.println(dur_walls[i]);
    }
}

// Print average and max durations.
static void print_timing_summary() {
    unsigned long sum_s = 0, sum_w = 0;
    uint16_t max_s = 0, max_w = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum_s += dur_service[i]; if (dur_service[i] > max_s) max_s = dur_service[i];
        sum_w += dur_walls[i];   if (dur_walls[i]   > max_w) max_w = dur_walls[i];
    }
    S.println();
    S.print("tof_service avg_us="); S.print(sum_s / NUM_SAMPLES);
    S.print(" max_us=");            S.println(max_s);
    S.print("wall_check avg_us=");  S.print(sum_w / NUM_SAMPLES);
    S.print(" max_us=");            S.println(max_w);
}

// =============================================================================

void setup() {
    S.begin(BAUD_RATE);
    delay(200);

    display_init();
    md.init();
    encoder_init();
    // imu_init();
    tof_init();

    delay(SETUP_DELAY_MS);
    S.println("starting");

    // ── the whole test, as a sequence of calls ──
    collect_samples();
    print_samples();
    print_timing_summary();
    S.println("DONE");
}

void loop() {}
