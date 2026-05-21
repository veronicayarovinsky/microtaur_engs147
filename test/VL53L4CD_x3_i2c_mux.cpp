// VL53L4CD_x3_i2c_mux.cpp

// TOF Sensor Unit Test - VL53L4CD characterization via PCA9548 I2C mux
// Purpose: measure accuracy, precision, range

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>

// config constants
#define MUX_ADDR        0x70
#define TOF_CH_LEFT     0
#define TOF_CH_FRONT    1
#define TOF_CH_RIGHT    2

#define BAUD_RATE       115200
#define NUM_SAMPLES     200
#define TS              100000UL    // 100 ms loop -> 10 Hz, µs

// Set this manually before each run; logged for offline analysis
#define KNOWN_DISTANCE_MM  100      // edit this for each test point

VL53L4CD sensor(&Wire, -1);

// select a single channel on the PCA9548 mux
void muxSelect(uint8_t ch) {
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(1 << ch);
    Wire.endTransmission();
}

// initialize the sensor currently selected on the mux
void initOneSensor() {
    sensor.InitSensor(0x29);
    sensor.VL53L4CD_SetRangeTiming(50, 0);  // 50 ms timing budget
    sensor.VL53L4CD_StartRanging();
}

// read one measurement (mm) from the currently selected channel
// returns -1 if invalid
int16_t readOneSensor() {
    uint8_t ready = 0;
    VL53L4CD_Result_t result;
    
    // wait for data ready (with timeout)
    unsigned long t0 = millis();
    while (!ready && (millis() - t0 < 200)) {
        sensor.VL53L4CD_CheckForDataReady(&ready);
    }
    if (!ready) return -1;
    
    sensor.VL53L4CD_GetResult(&result);
    sensor.VL53L4CD_ClearInterrupt();
    
    // status 0 = valid; statuses 1,2,3,4,6,7 are useful but flagged
    if (result.range_status != 0) return -1;
    return (int16_t)result.distance_mm;
}

void setup() {
    SerialUSB.begin(BAUD_RATE);
    Wire.begin();
    Wire.setClock(400000);
    delay(200);
    
    // initialize all three sensors (one at a time through the mux)
    uint8_t channels[3] = {TOF_CH_LEFT, TOF_CH_FRONT, TOF_CH_RIGHT};
    for (int i = 0; i < 3; i++) {
        muxSelect(channels[i]);
        delay(50);
        initOneSensor();
        delay(50);
    }
    
    delay(500);
}

void loop() {
    // storage: one row per sample, columns = [t, dL, dF, dR]
    unsigned long times[NUM_SAMPLES];
    int16_t dL[NUM_SAMPLES], dF[NUM_SAMPLES], dR[NUM_SAMPLES];
    
    unsigned long progstart, prevloopstart, curtime;
    bool running = false;
    unsigned long sampleidx = 0;
    
    progstart = micros();
    prevloopstart = progstart;
    curtime = progstart;
    
    while (sampleidx < NUM_SAMPLES) {
        curtime = micros();
        if (!running || (curtime - prevloopstart) >= TS) {
            // read each sensor in turn
            muxSelect(TOF_CH_LEFT);
            dL[sampleidx] = readOneSensor();
            
            muxSelect(TOF_CH_FRONT);
            dF[sampleidx] = readOneSensor();
            
            muxSelect(TOF_CH_RIGHT);
            dR[sampleidx] = readOneSensor();
            
            times[sampleidx] = curtime - progstart;
            
            running = true;
            prevloopstart = curtime;
            sampleidx++;
        }
    }
    
    // dump CSV
    SerialUSB.print("# Known distance (mm): ");
    SerialUSB.println(KNOWN_DISTANCE_MM);
    SerialUSB.println("Time(us),dL(mm),dF(mm),dR(mm)");
    for (unsigned long i = 0; i < sampleidx; i++) {
        SerialUSB.print(times[i]);
        SerialUSB.print(",");
        SerialUSB.print(dL[i]);
        SerialUSB.print(",");
        SerialUSB.print(dF[i]);
        SerialUSB.print(",");
        SerialUSB.println(dR[i]);
    }
    
    while(1){}
}

