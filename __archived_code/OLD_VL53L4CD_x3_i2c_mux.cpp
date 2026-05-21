/**
 * PCA9548 I2C Multi TOF Sensor Testing
 * VL53L4CD_x3_i2c_mux.cpp
 *
 * Using three VL53L4CD TOF sensors thru PCA9548 I2C Mux
 *
 */

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <Adafruit_BusIO_Register.h>

// config constants
#define PCAADDR         0x70
#define DEV_I2C         Wire
#define SerialPort      SerialUSB
#define BAUD_RATE       115200

#define TS              200000UL    // sample period: 200,000 us (200 ms) = 5 Hz frequency 
// #define RECORD_TIME     10000000UL  // run duration: 10 seconds (10,000,000 us)
#define NUM_SAMPLES     200

// TEST DISTANCE POINTS
#define L_TRUE_DIST_MM  100
#define R_TRUE_DIST_MM  100
#define F_TRUE_DIST_MM  100

#define PORT_L  3
#define PORT_R  0
#define PORT_F  1


// create sensor instances
VL53L4CD tof1(&DEV_I2C, A1);
VL53L4CD tof2(&DEV_I2C, A1);
VL53L4CD tof3(&DEV_I2C, A1);

// mux selection helper
void pcaselect(uint8_t i) {
    if (i > 7) return;
    Wire.beginTransmission(PCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();  
}

void setup() {
    while (!SerialPort);
    delay(1000);

    Wire.begin();
    SerialPort.begin(BAUD_RATE);
    // SerialPort.println("Initializing PCA9548 + VL53L4CD timed loop...");

    // define the port on the PCA9548 for sensor 1
    pcaselect(PORT_L);
    tof1.begin();
    tof1.VL53L4CD_Off();
    tof1.InitSensor();
    tof1.VL53L4CD_SetRangeTiming(200, 0); // 200ms timing budget
    tof1.VL53L4CD_StartRanging();

    // define the port on the PCA9548 for sensor 2
    pcaselect(PORT_R);
    tof2.begin();
    tof2.VL53L4CD_Off();
    tof2.InitSensor();
    tof2.VL53L4CD_SetRangeTiming(200, 0);
    tof2.VL53L4CD_StartRanging();

    // define the port on the PCA9548 for the 3rd sensor & setup sensor
    pcaselect(PORT_F);
    tof3.begin();
    tof3.VL53L4CD_Off();
    tof3.InitSensor();
    tof3.VL53L4CD_SetRangeTiming(200, 0);
    tof3.VL53L4CD_StartRanging();

    // SerialPort.println("Setup Complete. Starting Characterization...");
    delay(200);
}

void loop() {
    // Array declarations for data logging
    // const int num_samples = RECORD_TIME / TS;
    // unsigned long times[num_samples];
    unsigned long times[NUM_SAMPLES];
    uint16_t dist1[NUM_SAMPLES];
    uint16_t dist2[NUM_SAMPLES];
    uint16_t dist3[NUM_SAMPLES];

    // Variables for timing control
    unsigned long progstart, prevloopstart, curtime;
    unsigned long timeidx = 0;
    bool running = false;

    // Local result variables
    VL53L4CD_Result_t results;
    uint8_t NewDataReady = 0;

    curtime = micros();
    progstart = curtime;
    prevloopstart = curtime;

    // Main Timed Loop
    while (curtime < (progstart + RECORD_TIME) && (timeidx < num_samples)) {
        curtime = micros();

        // Enforce Loop Timing (TS)
        if (!running || (curtime - prevloopstart) >= TS) {
            
            // --- SENSOR 1 OPERATION ---
            pcaselect(PORT_L);
            // Non-blocking check for data
            tof1.VL53L4CD_CheckForDataReady(&NewDataReady);
            if (NewDataReady) {
                tof1.VL53L4CD_GetResult(&results);
                tof1.VL53L4CD_ClearInterrupt();
                dist1[timeidx] = results.distance_mm;
            } else {
                dist1[timeidx] = 0; // Data not ready
            }

            // --- SENSOR 2 OPERATION ---
            pcaselect(PORT_R);
            NewDataReady = 0;
            tof2.VL53L4CD_CheckForDataReady(&NewDataReady);
            if (NewDataReady) {
                tof2.VL53L4CD_GetResult(&results);
                tof2.VL53L4CD_ClearInterrupt();
                dist2[timeidx] = results.distance_mm;
            } else {
                dist2[timeidx] = 0;
            }

            // --- SENSOR 3 OPERATION ---
            pcaselect(PORT_F);
            NewDataReady = 0;
            tof3.VL53L4CD_CheckForDataReady(&NewDataReady);
            if (NewDataReady) {
                tof3.VL53L4CD_GetResult(&results);
                tof3.VL53L4CD_ClearInterrupt();
                dist3[timeidx] = results.distance_mm;
            } else {
                dist3[timeidx] = 0;
            }

            // Store timestamp and update loop states
            times[timeidx] = curtime - progstart;
            timeidx++;
            prevloopstart = curtime;
            running = true;
        }
    }

    // Print CSV Data to Serial Port
    SerialPort.println("Time(us),ToF1_Dist(mm),ToF2_Dist(mm),ToF3_Dist(mm)");
    for (int i = 0; i < timeidx; i++) {
        SerialPort.print(times[i]);
        SerialPort.print(",");
        SerialPort.print(dist1[i]);
        SerialPort.print(",");
        SerialPort.print(dist2[i]);
        SerialPort.print(",");
        SerialPort.println(dist3[i]);
    }

    // SerialPort.println("Testing Finished.");
    while(1) { 
        // Stop execution to prevent repeat loops
    }
}
