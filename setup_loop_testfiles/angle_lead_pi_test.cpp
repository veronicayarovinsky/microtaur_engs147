#include <Arduino.h>
#include <ArduinoMotorShieldR3.h>
#include <math.h>

#include "config.h"
#include "globals.h"
#include "imu.h"

#define S SerialUSB

ArduinoMotorShieldR3 md;

#define TARGET_DEG -90.0f
#define TS_US 5000UL
#define RECORD_TIME_US 5000000UL

// all of this was tuned in matlab :)
const float CTRL_B0 = 96.2709f;
const float CTRL_B1 = -188.7640f;
const float CTRL_B2 = 92.4932f;

const float CTRL_A1 = -1.7978f;
const float CTRL_A2 = 0.7978f;

const int PWM_MAX = 150;
const int PWM_STICTION = 95;

const float ANGLE_DEADBAND_RAD = 2.0f * PI / 180.0f;

float e0 = 0.0f, e1 = 0.0f, e2 = 0.0f;
float u0 = 0.0f, u1 = 0.0f, u2 = 0.0f;

static float wrap_pi(float a) {
    while (a > PI)  a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

int apply_pwm_limits(float u, float error_rad) {
    if (fabsf(error_rad) < ANGLE_DEADBAND_RAD) {
        return 0;
    }

    int pwm = (int)roundf(u);

    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;

    if (pwm > 0 && pwm < PWM_STICTION) pwm = PWM_STICTION;
    if (pwm < 0 && pwm > -PWM_STICTION) pwm = -PWM_STICTION;

    return pwm;
}

void setup() {
    S.begin(BAUD_RATE);

    md.init();
    imu_init();

    while (!S) {}

    while (!Micromouse::imu.ready) {
        imu_update();
        delay(50);
    }

    delay(500);

    S.println("time_s,target_deg,heading_deg,error_deg,u_raw,pwm_cmd");
}

void loop() {
    using namespace Micromouse;

    e0 = e1 = e2 = 0.0f;
    u0 = u1 = u2 = 0.0f;

    imu_update();
    float heading0 = imu.heading_rad;
    float target_rel = TARGET_DEG * PI / 180.0f;

    unsigned long start = micros();
    unsigned long prev = start;

    while (micros() - start < RECORD_TIME_US) {
        unsigned long now = micros();

        if (now - prev >= TS_US) {
            prev = now;

            imu_update();

            float heading_rel = wrap_pi(imu.heading_rad - heading0);

            e0 = wrap_pi(target_rel - heading_rel);

            u0 = CTRL_B0 * e0+ CTRL_B1 * e1+ CTRL_B2 * e2- CTRL_A1 * u1- CTRL_A2 * u2;

            int pwm_cmd = apply_pwm_limits(u0, e0);

            //  characterization showed +PWM,+PWM turns right/negative.
            // If the robot turns the wrong way, remove  negative sign
            int motor_pwm = -pwm_cmd;

            md.setM1Speed(motor_pwm);
            md.setM2Speed(motor_pwm);

            float t = (now - start) / 1000000.0f;

            S.print(t, 4); S.print(",");
            S.print(TARGET_DEG, 2); S.print(",");
            S.print(heading_rel * 180.0f / PI, 2); S.print(",");
            S.print(e0 * 180.0f / PI, 2); S.print(",");
            S.print(u0, 2); S.print(",");
            S.println(motor_pwm);

            e2 = e1;
            e1 = e0;

            u2 = u1;
            u1 = u0;
        }
    }

    md.setM1Speed(0);
    md.setM2Speed(0);

    while (1) {}
}