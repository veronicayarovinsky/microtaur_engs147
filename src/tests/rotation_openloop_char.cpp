// rotation_openloop_char.cpp
// https://micromouseonline.com/2011/05/15/characterising-the-drive-system-on-the-micromouse/

#include <Arduino.h>
#include <ArduinoMotorShieldR3.h>
#include <AxisEncoderShield3.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define HEADING_SOURCE 2                // 0 = encoders, 1 = IMU, 2 = both

#define PWM_COMMAND    150              // differential PWM magnitude (5-20% of 400 per Harrison)
                                        // positive = CCW spin (M1 fwd, M2 reverse)
#define BAUD_RATE      115200    
#define RECORD_TIME    2000000UL        // run duration, us
#define TS             5000UL           // loop period, us (5 ms = 200 Hz)

#define COUNTS_PER_REV 1440.0           // encoder counts per wheel revolution
#define WHEEL_RADIUS   0.016            // m (UPDATE to your value)
#define TRACK_WIDTH    0.080            // m, distance between wheel contact points (UPDATE)

#define BNO055_ADDR    0x28
#define BNO055_ID      55

#define USE_ENCODERS   (HEADING_SOURCE == 0 || HEADING_SOURCE == 2)
#define USE_IMU        (HEADING_SOURCE == 1 || HEADING_SOURCE == 2)

ArduinoMotorShieldR3 md;
#if USE_IMU
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_ID, BNO055_ADDR);
#endif


void setup() {
    SerialUSB.begin(BAUD_RATE);
    md.init();

#if USE_ENCODERS
    initEncoderShield();    //left = ch 1, right = ch 2
#endif

#if USE_IMU
    if (!bno.begin(OPERATION_MODE_NDOF)) {
        SerialUSB.println("ERROR: BNO055 not detected");
        while(1){}
    }
    bno.setExtCrystalUse(true);
#endif

    delay(1000);
}


void loop() {
    // array declarations & initialization
    const int num_samples = RECORD_TIME / TS;
    unsigned long times[num_samples];

#if USE_ENCODERS
    float headings_enc[num_samples];        // heading from encoder differential, rad
    float omegas_enc[num_samples];          // angular velocity from encoder diff, rad/s
#endif

#if USE_IMU
    float headings_imu[num_samples];        // unwrapped yaw from NDOF fusion, rad
    float omegas_imu[num_samples];          // gyro z-axis, rad/s
#endif

    // timing
    unsigned long progstart, prevloopstart, curtime;
    bool running = false;
    unsigned long timeidx = 0;

#if USE_ENCODERS
    long enc_L_curr = 0, enc_L_prev = 0;
    long enc_R_curr = 0, enc_R_prev = 0;
    float heading_enc = 0.0;
    float omega_enc = 0.0;
#endif

#if USE_IMU
    float heading_imu = 0.0;
    float omega_imu = 0.0;
    float yaw_initial_deg = 0.0;
    float yaw_prev_deg = 0.0;
    float yaw_wraps_deg = 0.0;
#endif

    // turn off motors, wait, then apply differential PWM step (spin in place)
    md.setM1Speed(0);
    md.setM2Speed(0);
    delay(1000);
    md.setM1Speed( PWM_COMMAND);   // differential mode: opposite signs
    md.setM2Speed(-PWM_COMMAND);   // produces pure rotation, zero forward motion

    // initialize timing variables
    curtime = micros();
    progstart = curtime;
    prevloopstart = curtime;
  
    while (curtime < (progstart + RECORD_TIME) && (timeidx < num_samples)) {
  
        // enforce loop timing
        curtime = micros();
        if (!running || (curtime - prevloopstart) >= TS) {
    
            // TIME CRITICAL OPERATIONS

#if USE_ENCODERS
            // read both encoders
            enc_L_curr = getEncoderValue(1);
            enc_R_curr = getEncoderValue(2);
#endif

#if USE_IMU
            // read NDOF-fused Euler orientation; .x() = yaw/heading in degrees [0, 360)
            imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
            float yaw_raw_deg = euler.x();
            // read gyro angular velocity (z-axis yaw rate, deg/s -> rad/s)
            imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
            omega_imu = gyro.z() * PI / 180.0;
#endif

            // compute heading and angular velocity from each enabled source
            if (!running) {
                // first-sample initialization: zero everything
#if USE_ENCODERS
                enc_L_prev = enc_L_curr;
                enc_R_prev = enc_R_curr;
                heading_enc = 0.0;
                omega_enc = 0.0;
#endif
#if USE_IMU
                yaw_initial_deg = yaw_raw_deg;
                yaw_prev_deg = yaw_raw_deg;
                yaw_wraps_deg = 0.0;
                heading_imu = 0.0;
#endif
            } else {
                float dt_sec = (float)(curtime - prevloopstart) / 1000000.0;

#if USE_ENCODERS
                // differential encoder kinematics:
                //   wheel arc difference = (delta_R - delta_L) * (2*pi*r / CPR)
                //   heading change       = arc / L
                float delta_L_counts = (float)(enc_L_curr - enc_L_prev);
                float delta_R_counts = (float)(enc_R_curr - enc_R_prev);
                float delta_L_m = (delta_L_counts / COUNTS_PER_REV) * 2.0 * PI * WHEEL_RADIUS;
                float delta_R_m = (delta_R_counts / COUNTS_PER_REV) * 2.0 * PI * WHEEL_RADIUS;
                float delta_heading = (delta_R_m - delta_L_m) / TRACK_WIDTH;   // rad
                heading_enc += delta_heading;
                omega_enc = delta_heading / dt_sec;
#endif

#if USE_IMU
                // unwrap fused yaw across the 0/360 boundary
                float delta_yaw_deg = yaw_raw_deg - yaw_prev_deg;
                if (delta_yaw_deg >  180.0) yaw_wraps_deg -= 360.0;
                if (delta_yaw_deg < -180.0) yaw_wraps_deg += 360.0;
                float unwrapped_deg = yaw_raw_deg + yaw_wraps_deg - yaw_initial_deg;
                heading_imu = unwrapped_deg * PI / 180.0;
#endif
            }

            // NOT TIME CRITICAL: store time and selected channels
            times[timeidx] = curtime - progstart;
#if USE_ENCODERS
            headings_enc[timeidx] = heading_enc;
            omegas_enc[timeidx]   = omega_enc;
#endif
#if USE_IMU
            headings_imu[timeidx] = heading_imu;
            omegas_imu[timeidx]   = omega_imu;
#endif

            // update states
#if USE_ENCODERS
            enc_L_prev = enc_L_curr;
            enc_R_prev = enc_R_curr;
#endif
#if USE_IMU
            yaw_prev_deg = yaw_raw_deg;
#endif
            running = true;
            prevloopstart = curtime;
            timeidx++;
        }
    
        // get current time for use in evaluating while() condition
        curtime = micros();
    }
  
    // stop motors
    md.setM1Speed(0);
    md.setM2Speed(0);
    
    // print all stored data to serial stream
#if HEADING_SOURCE == 0
    SerialUSB.println("Time(us),HeadingEnc(rad),OmegaEnc(rad/s)");
#elif HEADING_SOURCE == 1
    SerialUSB.println("Time(us),HeadingIMU(rad),OmegaIMU(rad/s)");
#else
    SerialUSB.println("Time(us),HeadingEnc(rad),OmegaEnc(rad/s),HeadingIMU(rad),OmegaIMU(rad/s)");
#endif

    for (unsigned int i = 0; i < timeidx; i++) {
        SerialUSB.print(times[i]);
#if USE_ENCODERS
        SerialUSB.print(",");
        SerialUSB.print(headings_enc[i], 6);
        SerialUSB.print(",");
        SerialUSB.print(omegas_enc[i], 4);
#endif
#if USE_IMU
        SerialUSB.print(",");
        SerialUSB.print(headings_imu[i], 6);
        SerialUSB.print(",");
        SerialUSB.print(omegas_imu[i], 4);
#endif
        SerialUSB.println();
    }

    while(1){}      // stops motor from running again
}
