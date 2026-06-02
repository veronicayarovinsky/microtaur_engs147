// angle_control_test_with_profile.cpp
// Turn-to-heading test for the trapezoid-profile angle loop (heading feedback only).
// Steps the FINAL target heading; the controller's internal velocity profile ramps,
// cruises, and decelerates the heading reference so the robot arrives with no overshoot.
// Logs final target, profile reference, and measured heading.

#include <Arduino.h>
#include <ArduinoMotorShieldR3.h>
#include <AxisEncoderShield3.h>
#include <math.h>
#include "config.h"
#include "globals.h"
#include "encoder.h"
#include "imu.h"
#include "controllers.h"

#define S            SerialUSB
#define N            600
#define RECORD_TIME  3000000UL
#define PRE_STEP_US  300000UL       // baseline before stepping the target
#define TARGET_DEG   90.0f          // heading step (+ = CCW)

ArduinoMotorShieldR3 md;

void setup() {
    S.begin(BAUD_RATE);
    md.init();
    encoder_init();
    imu_init();
    controllers_init();
    while (!S) {}
    while (!Micromouse::imu_data.ready) { imu_update(); delay(50); }
    delay(200);
}

void loop() {
    using namespace Micromouse;
    float t_s[N], tgt_deg[N], ref_deg[N], imu_deg[N], enc_deg[N];
    int   pwmL[N], pwmR[N];

    controllers_reset();
    encoder_reset_odometry();

    float head0 = imu_data.heading_rad;        // reference zero
    float step  = TARGET_DEG * PI / 180.0f;

    unsigned long now = micros(), prog = now, prev = now;
    int i = 0;
    while ((micros() - prog) <= RECORD_TIME && i < N) {
        now = micros();
        if ((now - prev) >= T_CONTROL_US) {
            uint32_t dt_us = now - prev; prev = now;
            imu_update();
            encoder_update(dt_us);

            float target = head0 + (((now - prog) >= PRE_STEP_US) ? step : 0.0f);
            update_controllers(encoders.dist_traveled_mm, target);  // isolate position, step heading

            float imu_d = (imu_data.heading_rad - head0) * 180.0f / PI;
            while (imu_d >  180.0f) imu_d -= 360.0f;
            while (imu_d < -180.0f) imu_d += 360.0f;

            t_s[i]     = (float)(now - prog) / 1e6f;
            tgt_deg[i] = (target - head0) * 180.0f / PI;
            ref_deg[i] = (controllers_get_theta_ref() - head0) * 180.0f / PI;
            imu_deg[i] = imu_d;
            enc_deg[i] = encoders.heading_rad * 180.0f / PI;
            pwmL[i]    = motors.pwm_left;
            pwmR[i]    = motors.pwm_right;
            ++i;
        }
    }
    md.setM1Speed(0); md.setM2Speed(0); controllers_reset();

    S.println("t_s,tgt_deg,ref_deg,imu_deg,enc_deg,pwmL,pwmR");
    for (int k = 0; k < i; ++k) {
        S.print(t_s[k],4);     S.print(','); S.print(tgt_deg[k],2); S.print(',');
        S.print(ref_deg[k],2); S.print(','); S.print(imu_deg[k],2); S.print(',');
        S.print(enc_deg[k],2); S.print(','); S.print(pwmL[k]);      S.print(',');
        S.println(pwmR[k]);
    }
    while (1) {}
}