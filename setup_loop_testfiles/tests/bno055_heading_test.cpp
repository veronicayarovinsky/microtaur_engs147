// BNO055 Heading Test

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// config constants
#define BAUD_RATE       115200
#define BNO055_ADDR     0x28        // matches proposal schematic (ADR pin HIGH)
#define SENSOR_ID       55
#define STARTUP_DELAY   3000        // ms - gives the operator time to step away
#define RECORD_TIME     5000000UL   // total record duration, us  (5 s)
#define TS              10000UL     // loop period, us  (10 ms -> 100 Hz)

// 1 = use offsets below. 0 to recalibrate
#define USE_SAVED_OFFSETS 0

#if USE_SAVED_OFFSETS
adafruit_bno055_offsets_t calibrationData = {
    0, 0, 0,    // accel x,y,z   <-- replace with values from calibration utility
    0, 0, 0,    // mag   x,y,z
    0, 0, 0,    // gyro  x,y,z
    1000,       // accel radius
    480         // mag   radius
};
#endif

Adafruit_BNO055 bno = Adafruit_BNO055(SENSOR_ID, BNO055_ADDR, &Wire1);

void setup() {
    SerialUSB.begin(BAUD_RATE);
    while (!SerialUSB) { delay(10); }

    SerialUSB.println("# BNO055 Heading Test");

    if (!bno.begin()) {
        SerialUSB.println("# ERROR: BNO055 not detected. Halting.");
        while (1) { delay(1000); }
    }

    delay(1000);
    bno.setExtCrystalUse(true);

#if USE_SAVED_OFFSETS
    bno.setSensorOffsets(calibrationData);
    SerialUSB.println("# Loaded saved calibration offsets.");
#endif

    // wait cuz gyro autocalibrates after a few seconds of being still
    SerialUSB.println("# Waiting for gyro calibration (keep sensor still)...");
    u_int8_t sys, gyro, accel, mag;
    unsigned long wait_start = millis();
    while (true) {
        bno.getCalibration(&sys, &gyro, &accel, &mag);
        if (gyro == 3) break;
        if (millis() - wait_start > 15000) {
            SerialUSB.println("# gyro not fully calibrated after 15 s. Proceeding anyway.");
            break;
        }
        delay(100);
    }

    SerialUSB.print("# Calibration status at start (sys,g,a,m): ");
    SerialUSB.print(sys); SerialUSB.print(",");
    SerialUSB.print(gyro); SerialUSB.print(",");
    SerialUSB.print(accel); SerialUSB.print(",");
    SerialUSB.println(mag);

    SerialUSB.print("# Starting recording in "); SerialUSB.print(STARTUP_DELAY);
    SerialUSB.println(" ms. Make sure sensor is at the reference angle and stationary.");
    delay(STARTUP_DELAY);
}

void loop() {
    // array declarations & initialization
    const int num_samples = RECORD_TIME / TS;
    unsigned long times[num_samples];
    float yaws[num_samples];
    float pitches[num_samples];
    float rolls[num_samples];
    float gyro_zs[num_samples];
    unsigned long read_durs[num_samples];
    u_int8_t cal_sys[num_samples];
    u_int8_t cal_gyr[num_samples];
    u_int8_t cal_acc[num_samples];
    u_int8_t cal_mag[num_samples];

    // variable declarations & initialization
    unsigned long progstart, prevloopstart, curtime;
    bool running = false;
    unsigned long timeidx = 0;

    sensors_event_t euler_event, gyro_event;

    // initialize timing variables
    curtime = micros();
    progstart = curtime;
    prevloopstart = curtime;

    while (curtime < (progstart + RECORD_TIME) && (timeidx < (unsigned long)num_samples)) {

        // enforce loop timing
        curtime = micros();
        if (!running || (curtime - prevloopstart) >= TS) {

            // TIME CRITICAL OPERATIONS
            // read sensor and measure the I2C transaction duration
            unsigned long read_start = micros();
            bno.getEvent(&euler_event, Adafruit_BNO055::VECTOR_EULER);
            bno.getEvent(&gyro_event,  Adafruit_BNO055::VECTOR_GYROSCOPE);
            unsigned long read_end = micros();

            // NOT TIME CRITICAL: store the sample
            times[timeidx]     = curtime - progstart;
            // Adafruit BNO055 maps Euler X -> heading/yaw (0->360 deg)
            yaws[timeidx]      = euler_event.orientation.x;
            pitches[timeidx]   = euler_event.orientation.y;
            rolls[timeidx]     = euler_event.orientation.z;
            // gyro event reports rad/s; Z is yaw rate
            gyro_zs[timeidx]   = gyro_event.gyro.z;
            read_durs[timeidx] = read_end - read_start;

            uint8_t s, g, a, m;
            bno.getCalibration(&s, &g, &a, &m);
            cal_sys[timeidx] = s;
            cal_gyr[timeidx] = g;
            cal_acc[timeidx] = a;
            cal_mag[timeidx] = m;

            // increment index & update states
            running = true;
            prevloopstart = curtime;
            timeidx++;
        }

        curtime = micros();
    }

    // print all stored data to serial
    SerialUSB.println("t_us,yaw_deg,pitch_deg,roll_deg,gyro_z_rps,read_us,sys,g,a,m");
    for (unsigned long i = 0; i < timeidx; i++) {
        SerialUSB.print(times[i]);             SerialUSB.print(",");
        SerialUSB.print(yaws[i], 4);           SerialUSB.print(",");
        SerialUSB.print(pitches[i], 4);        SerialUSB.print(",");
        SerialUSB.print(rolls[i], 4);          SerialUSB.print(",");
        SerialUSB.print(gyro_zs[i], 6);        SerialUSB.print(",");
        SerialUSB.print(read_durs[i]);         SerialUSB.print(",");
        SerialUSB.print(cal_sys[i]);           SerialUSB.print(",");
        SerialUSB.print(cal_gyr[i]);           SerialUSB.print(",");
        SerialUSB.print(cal_acc[i]);           SerialUSB.print(",");
        SerialUSB.println(cal_mag[i]);
    }

    // summary
    double yaw_sum = 0.0, read_sum = 0.0;
    unsigned long read_max = 0, read_min = 0xFFFFFFFFUL;
    float yaw_min = 360.0f, yaw_max = 0.0f;
    for (unsigned long i = 0; i < timeidx; i++) {
        yaw_sum  += yaws[i];
        read_sum += read_durs[i];
        if (read_durs[i] > read_max) read_max = read_durs[i];
        if (read_durs[i] < read_min) read_min = read_durs[i];
        if (yaws[i] > yaw_max)       yaw_max  = yaws[i];
        if (yaws[i] < yaw_min)       yaw_min  = yaws[i];
    }
    SerialUSB.println();
    SerialUSB.println("# ---- summary ----");
    SerialUSB.print("# samples:        "); SerialUSB.println(timeidx);
    SerialUSB.print("# yaw mean (deg): "); SerialUSB.println(yaw_sum / (double)timeidx, 4);
    SerialUSB.print("# yaw min  (deg): "); SerialUSB.println(yaw_min, 4);
    SerialUSB.print("# yaw max  (deg): "); SerialUSB.println(yaw_max, 4);
    SerialUSB.print("# yaw p-p  (deg): "); SerialUSB.println(yaw_max - yaw_min, 4);
    SerialUSB.print("# read mean (us): "); SerialUSB.println(read_sum / (double)timeidx, 2);
    SerialUSB.print("# read min  (us): "); SerialUSB.println(read_min);
    SerialUSB.print("# read max  (us): "); SerialUSB.println(read_max);
    SerialUSB.println("# -----------------");

    while (1) {}    // halt
}
