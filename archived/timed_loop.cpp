// Sample code for implementing a timed loop on the Arduino Due platform
// Author: M. Kokko
// Updated: 08-Apr-2025

// you may need some additional #include or #define directives here
#include <Arduino.h>

// you will need to do some initialization in setup()
void setup() {
  Serial.begin(9600);  // change baud rate if desired
}

void loop() {

  // variables (keep declarations here unless you *REALLY* need a global...)
  unsigned long progstart, prevloopstart, curtime, dt;  // note: long is a long INTEGER!
  bool running = false;                                 // allows us to enter TS if() block on first run through loop

  // turn off motor, wait for some period of time, then set motor to desired speed
  // collect data for two seconds
  curtime = micros();
  progstart = curtime;
  prevloopstart = curtime;
  unsigned long times[2000];
  unsigned long timeidx = 0;

  while (curtime < (progstart + RECORD_TIME)) {  // adjust for other ending conditions

    // enforce loop timing
    curtime = micros();
    if (!running || (curtime - prevloopstart) >= TS) {

      // TIME CRITICAL OPERATIONS
      // * read encoder
      // * compute velocity (define v = 0 at first timestep, i.e. when running == false)

      // store time and position
      // increment indices, pointers, etc.
      // and take care of other items that are not time critical
      running = true;
      prevloopstart = curtime;  // absolute count of when this sampling period started

      // check for array overruns if desired

      // check for sample time overrun if desired
    }

    // get current time for use in evaluating while() condition
    curtime = micros();
  }

  // stop motor
  // print all stored data to serial stream
}
