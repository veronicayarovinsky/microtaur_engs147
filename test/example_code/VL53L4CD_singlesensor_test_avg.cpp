/**
 * Single VL53L4CD Sensor - Timed Loop with Moving Average
 */

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>

// Hardware Constants
#define DEV_I2C         Wire
#define SerialPort      SerialUSB
#define BAUD_RATE       115200

// Timing & Sampling Configuration
#define TS              200000UL    // Sample period: 200,000 µs (200ms)
#define NUM_SAMPLES     100         // Exact number of consecutive samples to record
#define MA_WINDOW       5           // Number of samples for the moving average window

// Create single sensor instance
VL53L4CD tof(&DEV_I2C, A1);

void setup() {
    while (!SerialPort);
    delay(1000);

    Wire.begin();
    SerialPort.begin(BAUD_RATE);
    SerialPort.println("Initializing VL53L4CD Timed Loop...");

    // Setup Sensor
    tof.begin();
    tof.VL53L4CD_Off();
    tof.InitSensor();
    tof.VL53L4CD_SetRangeTiming(200, 0); // 200ms timing budget to match TS
    tof.VL53L4CD_StartRanging();

    SerialPort.println("Setup Complete. Starting 100-Sample Characterization...");
    delay(200);
}

void loop() {
    // Arrays for data logging
    unsigned long times[NUM_SAMPLES];
    uint16_t distances[NUM_SAMPLES];
    float moving_averages[NUM_SAMPLES];

    // Timing control variables
    unsigned long progstart, prevloopstart, curtime;
    unsigned int timeidx = 0;
    bool running = false;

    // Sensor result variables
    VL53L4CD_Result_t results;
    uint8_t NewDataReady = 0;

    curtime = micros();
    progstart = curtime;
    prevloopstart = curtime;

    // Main Timed Loop: runs exactly NUM_SAMPLES times
    while (timeidx < NUM_SAMPLES) {
        curtime = micros();

        // Enforce Loop Timing (TS)
        if (!running || (curtime - prevloopstart) >= TS) {
            
            // Poll sensor
            NewDataReady = 0;
            tof.VL53L4CD_CheckForDataReady(&NewDataReady);
            
            if (NewDataReady) {
                tof.VL53L4CD_GetResult(&results);
                tof.VL53L4CD_ClearInterrupt();
                distances[timeidx] = results.distance_mm;
            } else {
                // If data is missed due to jitter, carry over previous value to maintain data integrity
                distances[timeidx] = (timeidx > 0) ? distances[timeidx - 1] : 0; 
            }

            // Calculate moving average
            uint32_t sum = 0;
            uint8_t count = 0;
            
            // Look back through the array up to the MA_WINDOW size
            for (int i = 0; i < MA_WINDOW; i++) {
                if ((int)timeidx - i >= 0) {
                    sum += distances[timeidx - i];
                    count++;
                }
            }
            // Store calculated average
            moving_averages[timeidx] = (float)sum / count;

            // Store timestamp and update loop states
            times[timeidx] = curtime - progstart;
            timeidx++;
            prevloopstart = curtime;
            running = true;
        }
    }

    // Print CSV Data to Serial Port
    SerialPort.println("Time(us),Distance(mm),MovingAverage(mm)");
    for (int i = 0; i < NUM_SAMPLES; i++) {
        SerialPort.print(times[i]);
        SerialPort.print(",");
        SerialPort.print(distances[i]);
        SerialPort.print(",");
        SerialPort.println(moving_averages[i], 2); // Print float with 2 decimal places
    }

    SerialPort.println("Testing Finished.");
    
    // Stop execution
    while(1) {}
}
