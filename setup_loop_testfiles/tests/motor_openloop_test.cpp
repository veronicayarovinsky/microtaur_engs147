// Open-loop Motor Characterization - timed loop on the Arduino Due platform
// Purpose: record time, position, and speed data from encoder when motor responds to a step input

#include <Arduino.h>
#include <ArduinoMotorShieldR3.h>
#include <AxisEncoderShield3.h>
#include <vl53l4cd_class.h>
#include <Adafruit_BusIO_Register.h>

// https://www.pololu.com/product/5217

// config constants
#define PWM_COMMAND                 140
// #define COUNTS_PER_REV 1440.0
#define GEARBOX_REDUCTION_FACTOR    100.37
#define COUNTS_PER_REV              (12 * GEARBOX_REDUCTION_FACTOR)
#define BAUD_RATE                   115200    
#define RECORD_TIME                 10000000UL      // run duration, µs
#define TS                          5000UL      // loop timing, µs 

// #ifndef PWM_COMMAND
// #define PWM_COMMAND                 100             // Default test value
// #endif

ArduinoMotorShieldR3 md;

void setup() {
    SerialUSB.begin(BAUD_RATE);  // initialize serial connection
    md.init();                // initialize motor shield
    initEncoderShield();      // initialize encoder shield to read one channel

    // ensure all configuration is complete
    delay(200);
}


void loop() {
    // array declarations & initialization
    const int num_samples = RECORD_TIME / TS;
    unsigned long times[num_samples];
    long positions[num_samples];
    float speeds[num_samples];

    // variable declarations & initialization
    unsigned long progstart, prevloopstart, curtime, dt;  // note: long is a long INTEGER!
    bool running = false;                                 // allows us to enter TS if() block on first run through loop
    long current_position = 0, previous_position = 0;
    float current_speed = 0.0;
    unsigned long timeidx = 0;

    // turn off motor, wait for some period of time, then set motor to desired speed
    md.setM1Speed(0);
    delay(1000);
    md.setM1Speed(PWM_COMMAND);

    // initialize timing variables
    curtime = micros();
    progstart = curtime;
    prevloopstart = curtime;
  
    while (curtime < (progstart + RECORD_TIME) && (timeidx < num_samples)) {  // adjust for other ending conditions
  
        // enforce loop timing
        curtime = micros();
        if (!running || (curtime - prevloopstart) >= TS) {
    
            // TIME CRITICAL OPERATIONS
            // * read encoder
            current_position = getEncoderValue(1);
            // * compute velocity (define v = 0 at first timestep, i.e. when running == false)
            if (!running) {
                current_speed = 0.0; 
            } else {
                // calculate speed in rad/s
                float delta_pos = (float)(current_position - previous_position);
                float delta_time_sec = (float)(curtime - prevloopstart) / 1000000.0;
                current_speed = -(delta_pos / COUNTS_PER_REV) * 2.0 * PI / delta_time_sec;
            }

            // NOT TIME CRITICAL: store time, position, speed
            times[timeidx] = curtime - progstart; // store relative timestamp
            positions[timeidx] = current_position;
            speeds[timeidx] = current_speed;

            // increment indeces & pointers; update states
            previous_position = current_position;
            running = true;
            prevloopstart = curtime;        // absolute count of when this sampling period started
            timeidx++;
        }
    
        // get current time for use in evaluating while() condition
        curtime = micros();
    }
  
    // stop motor
    md.setM1Speed(0);
    
    // print all stored data to serial stream
    SerialUSB.println("Time(us),Position(counts),Speed(rad/s)");
    for (int i = 0; i < timeidx; i++) {
        SerialUSB.print(times[i]);
        SerialUSB.print(",");
        SerialUSB.print(positions[i]);
        SerialUSB.print(",");
        SerialUSB.println(speeds[i], 4);
    }
    // SerialUSB.println("EOF");
    while(1){}      // stops motor from running again
}
