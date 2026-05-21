#include <Arduino.h>
#include <ArduinoMotorShieldR3.h>
#include "globals.h"
#include "encoder.h"
#include "imu.h"
#include "tof.h"
#include "motor_control.h"
#include "heading_control.h"
#include "lateral_control.h"
#include "drive.h"
#include "config.h"

using namespace Micromouse;
ArduinoMotorShieldR3 md;

static const int N = 1;
static unsigned long times_us[N];
static float hdg_ref[N], hdg_meas[N], hdg_err[N];
static int   turn_num[N], pwm_L[N], pwm_R[N];

void loop() {
    md.init();
    encoder_init(); imu_init();
    drive_init();

    while (!imu.ready) { imu_update(); delay(50); }
    float initial_heading = imu.heading_rad;

    unsigned long progstart  = micros();
    unsigned long t_motors   = progstart;
    unsigned long t_imu_last = progstart;
    bool running  = false;
    int  turn     = 0;
    int  idx      = 0;
    bool pausing  = false;
    unsigned long pause_ms = 0;

    SerialUSB.print("Initial: "); SerialUSB.print(initial_heading*180/PI, 2); SerialUSB.println(" deg");

    while (turn < 4 && idx < N) {
        unsigned long now = micros();

        if (pausing) {
            if (millis() - pause_ms >= 500) pausing = false;
            continue;
        }

        // ── 1ms ──────────────────────────────────────────────────────────
        if (!running || (now - t_motors) >= T_CONTROL_US) {
            uint32_t dt = running ? now - t_motors : T_CONTROL_US;
            t_motors    = running ? t_motors + T_CONTROL_US : now;
            encoder_update(dt);
            motor_pi_update(heading_pd_get_omega_left(), heading_pd_get_omega_right());
            md.setM1Speed(motors.pwm_left);
            md.setM2Speed(motors.pwm_right);
        }

        // ── 10ms ─────────────────────────────────────────────────────────
        if (!running || (now - t_imu_last) >= T_IMU_US) {
            t_imu_last = running ? t_imu_last + T_IMU_US : now;
            running    = true;
            imu_update();
            heading_pd_update(intent.heading_ref_rad, intent.v_base_m_s, 0.0f);
            // Log
            float ref = intent.heading_ref_rad;
            float err = (ref - imu.heading_rad) * 180.f / PI;
            times_us[idx]  = now - progstart;
            hdg_ref[idx]   = ref * 180.f / PI;
            hdg_meas[idx]  = imu.heading_rad * 180.f / PI;
            hdg_err[idx]   = err;
            turn_num[idx]  = turn + 1;
            pwm_L[idx]     = motors.pwm_left;
            pwm_R[idx]     = motors.pwm_right;
            idx++;
        }

        // ── Drive primitive ───────────────────────────────────────────────
        if (drive_turn(PI / 2.0f)) {
            turn++;
            float err = (intent.heading_ref_rad - imu.heading_rad) * 180.f / PI;
            SerialUSB.print("Turn "); SerialUSB.print(turn);
            SerialUSB.print(": "); SerialUSB.print(imu.heading_rad*180/PI, 2);
            SerialUSB.print(" deg  err="); SerialUSB.print(err, 2); SerialUSB.println(" deg");
            pausing   = true;
            pause_ms  = millis();
        }
    }

    md.setM1Speed(0); md.setM2Speed(0);
    float cumulative = (initial_heading - imu.heading_rad) * 180.f / PI;
    SerialUSB.print("Cumulative error after 4×90°: "); SerialUSB.print(cumulative,2); SerialUSB.println(" deg");

    SerialUSB.println("time_us,turn,hdg_ref_deg,hdg_meas_deg,hdg_err_deg,pwm_L,pwm_R");
    for (int i = 0; i < idx; i++) {
        SerialUSB.print(times_us[i]);   SerialUSB.print(",");
        SerialUSB.print(turn_num[i]);   SerialUSB.print(",");
        SerialUSB.print(hdg_ref[i],2);  SerialUSB.print(",");
        SerialUSB.print(hdg_meas[i],2); SerialUSB.print(",");
        SerialUSB.print(hdg_err[i],2);  SerialUSB.print(",");
        SerialUSB.print(pwm_L[i]);      SerialUSB.print(",");
        SerialUSB.println(pwm_R[i]);
    }
    while (1) {}
}

