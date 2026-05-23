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

enum Step { FWD1, TURN_L, FWD2, TURN_R, FWD3, FINISHED };
static Step s_step = FWD1;

void loop() {
    md.init();
    encoder_init(); imu_init(); tof_init();
    drive_init();

    while (!imu.ready) { imu_update(); delay(50); }

    unsigned long t_motors   = micros();
    unsigned long t_imu_last = micros();
    unsigned long t_tof_last = micros();
    bool running = false;

    SerialUSB.println("step,event,dist_cm,heading_deg");

    while (s_step != FINISHED) {
        unsigned long now = micros();

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
            heading_pd_update(drive_command.heading_ref_rad, drive_command.v_base_m_s,
                              lateral_pd_get_correction());
        }

        // ── 15ms ─────────────────────────────────────────────────────────
        if (!running || (now - t_tof_last) >= T_TOF_US) {
            t_tof_last = running ? t_tof_last + T_TOF_US : now;
            tof_trigger();
        }
        if (tof_read_if_ready()) lateral_pd_update();

        // ── Sequence state machine ────────────────────────────────────────
        bool done = false;
        switch (s_step) {
            case FWD1:   done = drive_forward(2 * CELL_SIZE_M, SPEED_NOMINAL); break;
            case TURN_L: done = drive_turn( PI / 2.0f);                    break;
            case FWD2:   done = drive_forward(2 * CELL_SIZE_M, SPEED_NOMINAL); break;
            case TURN_R: done = drive_turn(-PI / 2.0f);                    break;
            case FWD3:   done = drive_forward(    CELL_SIZE_M, SPEED_NOMINAL); break;
            default: break;
        }
        if (done) {
            const char* names[] = {"FWD1","TURN_L","FWD2","TURN_R","FWD3"};
            SerialUSB.print(names[(int)s_step]); SerialUSB.print(",done,");
            SerialUSB.print(encoders.dist_traveled_m * 100.f, 1); SerialUSB.print(",");
            SerialUSB.println(imu.heading_rad * 180.f / PI, 1);
            s_step = (Step)((int)s_step + 1);
        }
    }

    md.setM1Speed(0); md.setM2Speed(0);
    SerialUSB.println("Sequence complete.");
    while (1) {}
}
