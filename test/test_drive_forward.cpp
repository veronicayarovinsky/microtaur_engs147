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

constexpr int NUM_CELLS = 5;

using namespace Micromouse;
ArduinoMotorShieldR3 md;

static const int N = 1;
static unsigned long times_us[N];
static float dist_m[N], hdg_deg[N], hdg_err_deg[N];
static int   cell_num[N], pwm_L[N], pwm_R[N];

void loop() {
    md.init();
    encoder_init(); imu_init(); tof_init();
    drive_init();

    while (!imu.ready) { imu_update(); delay(50); }
    float heading_ref = imu.heading_rad;

    unsigned long progstart   = micros();
    unsigned long t_motors    = progstart;
    unsigned long t_imu_last  = progstart;
    unsigned long t_tof_last  = progstart;
    bool running = false;
    int  cells   = 0;
    int  idx     = 0;

    while (cells < NUM_CELLS && idx < N) {
        unsigned long now = micros();

        // ── 1ms: encoder → motor PI → apply PWM ──────────────────────────
        if (!running || (now - t_motors) >= T_CONTROL_US) {
            uint32_t dt = running ? now - t_motors : T_CONTROL_US;
            t_motors    = running ? t_motors + T_CONTROL_US : now;
            encoder_update(dt);
            motor_pi_update(heading_pd_get_omega_left(), heading_pd_get_omega_right());
            md.setM1Speed(motors.pwm_left);
            md.setM2Speed(motors.pwm_right);
        }

        // ── 10ms: IMU → heading PD + log ─────────────────────────────────
        if (!running || (now - t_imu_last) >= T_IMU_US) {
            t_imu_last = running ? t_imu_last + T_IMU_US : now;
            running    = true;
            imu_update();
            heading_pd_update(intent.heading_ref_rad, intent.v_base_m_s,
                              lateral_pd_get_correction());
            // Log at 10ms rate
            float he = (heading_ref - imu.heading_rad) * 180.f / PI;
            times_us[idx]    = now - progstart;
            dist_m[idx]      = encoders.dist_traveled_m;
            hdg_deg[idx]     = imu.heading_rad * 180.f / PI;
            hdg_err_deg[idx] = he;
            cell_num[idx]    = cells;
            pwm_L[idx]       = motors.pwm_left;
            pwm_R[idx]       = motors.pwm_right;
            idx++;
        }

        // ── 15ms: TOF + lateral PD ────────────────────────────────────────
        if (!running || (now - t_tof_last) >= T_TOF_US) {
            t_tof_last = running ? t_tof_last + T_TOF_US : now;
            tof_trigger();
        }
        if (tof_read_if_ready()) lateral_pd_update();

        // ── Drive primitive ───────────────────────────────────────────────
        if (drive_forward(CELL_SIZE_M, V_EXPLORE)) {
            cells++;
            SerialUSB.print("Cell "); SerialUSB.print(cells);
            SerialUSB.print(" done. dist=");
            SerialUSB.print(encoders.dist_traveled_m * 100.f, 1); SerialUSB.println("cm");
        }
    }

    md.setM1Speed(0); md.setM2Speed(0);

    SerialUSB.println("time_us,cell,dist_m,hdg_deg,hdg_err_deg,pwm_L,pwm_R");
    for (int i = 0; i < idx; i++) {
        SerialUSB.print(times_us[i]);      SerialUSB.print(",");
        SerialUSB.print(cell_num[i]);      SerialUSB.print(",");
        SerialUSB.print(dist_m[i], 4);     SerialUSB.print(",");
        SerialUSB.print(hdg_deg[i], 2);    SerialUSB.print(",");
        SerialUSB.print(hdg_err_deg[i],2); SerialUSB.print(",");
        SerialUSB.print(pwm_L[i]);         SerialUSB.print(",");
        SerialUSB.println(pwm_R[i]);
    }
    while (1) {}
}
