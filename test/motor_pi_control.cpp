// Discrete PI control law (parallel form, backward-Euler integration):
//     e[k]       = omega_ref - omega_meas[k]
//     I[k]       = I[k-1] + e[k] * Ts_sec       (only updated if not saturated)
//     u_volt[k]  = Kp * e[k] + Ki * I[k]
//     pwm[k]     = voltage_to_pwm(u_volt[k])


#include <Arduino.h>
#include <ArduinoMotorShieldR3.h>
#include <AxisEncoderShield3.h>
#include "voltage_to_pwm.cpp"

// --------- reference & gains ---------
#define REF_SPEED       100.0f      // reference speed [rad/s] -- match what you tuned around in lab2
#define KP              0.08f       // proportional gain [V / (rad/s)]
#define KI              0.40f       // integral gain    [V / (rad/s * s)] -- START HERE, then tune
                                    // (rule of thumb: Ki ~ Kp / tau_motor; adjust based on response)

// --------- saturation limits for anti-windup ---------
// Matches the voltage_to_pwm() table endpoints. If |u_volt| would exceed this
// AND error is pushing it further in the same direction, freeze the integrator.
#define U_VOLT_MAX      8.25f
#define U_VOLT_MIN     -8.26f

// --------- platform constants ---------
#define GEARBOX_REDUCTION_FACTOR    100.37f
#define COUNTS_PER_REV              (12.0f * GEARBOX_REDUCTION_FACTOR)
#define BAUD_RATE                   115200
#define RECORD_TIME                 10000000UL      // run duration [us]
#define TS                          5000UL          // loop period  [us]
#define TS_SEC                      (TS / 1000000.0f)   // loop period [s]

ArduinoMotorShieldR3 md;

void setup() {
    SerialUSB.begin(BAUD_RATE);
    md.init();
    initEncoderShield();
    delay(200);
}

void loop() {
    // ----- logging buffers -----
    const int num_samples = RECORD_TIME / TS;
    unsigned long times[num_samples];
    float         speeds[num_samples];
    float         errors[num_samples];
    float         integrals[num_samples];
    float         voltages[num_samples];
    long          pwms[num_samples];

    // ----- loop-state variables -----
    unsigned long progstart, prevloopstart, curtime;
    bool running = false;
    long current_position = 0, previous_position = 0;
    float current_speed = 0.0f;
    float error = 0.0f;
    float integral = 0.0f;          // integrator state, units of (rad/s)*s = rad
    float voltage_cmd = 0.0f;
    long  pwm_cmd = 0;
    unsigned long timeidx = 0;

    // ----- ensure motor is stopped before starting -----
    md.setM1Speed(0);
    delay(1000);

    // ----- initialize timing -----
    curtime = micros();
    progstart = curtime;
    prevloopstart = curtime;

    while (curtime < (progstart + RECORD_TIME) && (timeidx < num_samples)) {

        // enforce loop timing
        curtime = micros();
        if (!running || (curtime - prevloopstart) >= TS) {

            // ===== TIME-CRITICAL =====

            // read encoder
            current_position = getEncoderValue(1);

            // compute measured speed [rad/s]
            //    (define v=0 at first timestep, when running==false)
            if (!running) {
                current_speed = 0.0f;
            } else {
                float delta_pos      = (float)(current_position - previous_position);
                float delta_time_sec = (float)(curtime - prevloopstart) / 1000000.0f;
                // sign convention matches lab2 (negative because of encoder direction vs commanded direction)
                current_speed = -1.0f * (delta_pos / COUNTS_PER_REV) * 2.0f * PI / delta_time_sec;
            }

            // PI control
            error = REF_SPEED - current_speed;

            // update integrator
            float integral_candidate = integral + error * TS_SEC;
            float voltage_candidate  = KP * error + KI * integral_candidate;

            // only commit integrator update if the resulting command is unsaturated
            // OR error is pulling it out of saturation
            bool saturated_high = (voltage_candidate >  U_VOLT_MAX) && (error > 0);
            bool saturated_low  = (voltage_candidate <  U_VOLT_MIN) && (error < 0);
            if (!saturated_high && !saturated_low) {
                integral = integral_candidate;
            }
            voltage_cmd = KP * error + KI * integral;

            // convert to PWM and command motor
            pwm_cmd = voltage_to_pwm(voltage_cmd);
            md.setM1Speed(pwm_cmd);

            // ===== NOT TIME-CRITICAL: log to buffers =====
            times[timeidx]     = curtime - progstart;
            speeds[timeidx]    = current_speed;
            errors[timeidx]    = error;
            integrals[timeidx] = integral;
            voltages[timeidx]  = voltage_cmd;
            pwms[timeidx]      = pwm_cmd;

            // update state for next iteration
            previous_position = current_position;
            running = true;
            prevloopstart = curtime;
            timeidx++;
        }

        curtime = micros();
    }

    // stop motor
    md.setM1Speed(0);

    // ----- dump everything to serial -----
    SerialUSB.println("Time(us),Speed(rad/s),Error(rad/s),Integral(rad),Voltage(V),PWM");
    for (unsigned long i = 0; i < timeidx; i++) {
        SerialUSB.print(times[i]);
        SerialUSB.print(",");
        SerialUSB.print(speeds[i], 3);
        SerialUSB.print(",");
        SerialUSB.print(errors[i], 3);
        SerialUSB.print(",");
        SerialUSB.print(integrals[i], 4);
        SerialUSB.print(",");
        SerialUSB.print(voltages[i], 3);
        SerialUSB.print(",");
        SerialUSB.println(pwms[i]);
    }

    while (1) {}    // halt
}
