#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include "config.h"
#include "globals.h"
#include "tof.h"

// tof.cpp
// each sensor ranges continuously --> new measurement every ~10ms
// tof_try_read() checks the data-ready flag ONCE (no wait loop)
// if ready, it reads and stores the value; if not, returns
// tof_service() cycles through the sensors one per call
// 

#define I2C_TOF         Wire1
#define S               SerialUSB
#define DEBUG_MODE      0       // change to 1 to enable debugging outputs
#define CALIBRATE_OFFSET    0

// sensor objects
// -1 passed into XSHUT arg of sensor object constructor --> disables XSHUT functionality
static VL53L4CD tof_left      (&I2C_TOF, -1);
static VL53L4CD tof_diag_left (&I2C_TOF, -1);
static VL53L4CD tof_front     (&I2C_TOF, -1);
static VL53L4CD tof_diag_right(&I2C_TOF, -1);
static VL53L4CD tof_right     (&I2C_TOF, -1);


static VL53L4CD* const sensors[NUM_TOF_SENSORS] = {
    &tof_left, &tof_diag_left, &tof_front, &tof_diag_right, &tof_right
};
 
static const uint8_t channels[NUM_TOF_SENSORS] = {
    TOF_CH_LEFT, TOF_CH_DIAG_L, TOF_CH_FRONT, TOF_CH_DIAG_R, TOF_CH_RIGHT
};

static const int16_t tof_offsets[NUM_TOF_SENSORS] = {-13, -21, -25, -26, -25};
 
// Pointer to the Micromouse::tof field each sensor writes to
static int16_t* dist_field[NUM_TOF_SENSORS];
 
// Per-sensor freshness flags
static volatile bool s_fresh[NUM_TOF_SENSORS] = { false };
 
// Per-sensor last range_status
static uint8_t s_status[NUM_TOF_SENSORS] = { 0 };

// service index
static TofId s_next = TOF_LEFT;


// select a single channel on the PCA9548 mux
static void mux_select(uint8_t ch) {
    I2C_TOF.beginTransmission(0x70);
    I2C_TOF.write(1 << ch);
    // I2C_TOF.endTransmission();
    uint8_t err = I2C_TOF.endTransmission();
    #if DEBUG_MODE == 1
        S.print("mux_select ch="); S.print(ch);
        S.print(" endTransmission err="); S.println(err);   // 0 = mux ACKed
    #endif
}
static void mux_disable() {
    I2C_TOF.beginTransmission(0x70);
    I2C_TOF.write(0x00);
    I2C_TOF.endTransmission();
}

static void init_one_sensor(VL53L4CD* sensor, uint8_t channel, uint16_t offset) {
    mux_select(channel);
    #if DEBUG_MODE == 1
        I2C_TOF.beginTransmission(0x29);
        uint8_t sensor_err = I2C_TOF.endTransmission();
        S.print("sensor 0x29 probe err="); S.println(sensor_err);  // 0 = sensor present
    #endif

    uint8_t status = sensor->InitSensor();
    if (status != 0) { S.print("InitSensor failed, channel "); S.println(channel); return; }
    
    sensor->VL53L4CD_SetRangeTiming(TOF_RANGE_TIMING_BUDGET_MS, 0);   // 10ms timing budget

    #if CALIBRATE_OFFSET == 1
        int16_t offset = 0;
        sensor->VL53L4CD_CalibrateOffset(100, &offset, 20);   // target flat at 100mm
        S.print("ch "); S.print(channel); S.print(" offset="); S.println(offset);
    #else
        sensor->VL53L4CD_SetOffset(offset);
    #endif
    sensor->VL53L4CD_StartRanging();          // continuous mode
}


void tof_init() {
    using namespace Micromouse;
    I2C_TOF.begin();
    I2C_TOF.setClock(400000);      // 400 kHz
    dist_field[TOF_LEFT]   = &tof.dist_left_mm;
    dist_field[TOF_DIAG_L] = &tof.dist_diag_left_mm;
    dist_field[TOF_FRONT]  = &tof.dist_front_mm;
    dist_field[TOF_DIAG_R] = &tof.dist_diag_right_mm;
    dist_field[TOF_RIGHT]  = &tof.dist_right_mm;

    for (int i = 0; i < TOF_COUNT; i++) {
        s_fresh[i] = false;
        delay(50);
        // Serial.print("Attempting to init sensor on channel: ");
        // Serial.println(channels[i]);

        init_one_sensor(sensors[i], channels[i], tof_offsets[i]);
        delay(50);
    }
    s_next = TOF_LEFT;
}

int tof_try_read(TofId id) {
    VL53L4CD* sensor = sensors[id];
    mux_select(channels[id]);

    uint8_t ready = 0;
    sensor->VL53L4CD_CheckForDataReady(&ready);
    if (!ready) return 0;                 // not ready — return immediately, no wait

    VL53L4CD_Result_t result;
    sensor->VL53L4CD_GetResult(&result);
    sensor->VL53L4CD_ClearInterrupt();

    s_status[id] = result.range_status;        // get status of reading

    if (result.range_status != 0) return -1;   // invalid

    *dist_field[id] = (int16_t)result.distance_mm;
    s_fresh[id] = true;
    return 1;
}

void tof_service() {
    int r = tof_try_read(s_next);
    if (r != 0) {                                   // got a result → advance
        s_next = (TofId)((s_next + 1) % TOF_COUNT);
    }
    // r == 0 (not ready): stay on this sensor, try again next tick
}

bool tof_is_fresh(TofId id)       { return s_fresh[id]; }
void tof_consume_fresh(TofId id)  { s_fresh[id] = false; }

// Accessor:
uint8_t tof_get_status(TofId id) { return s_status[id]; }

// Wall detection
void tof_check_walls_current_cell() {
    using namespace Micromouse;
    walls_current_cell.left = tof.dist_left_mm < WALL_PRESENT_CURRENT_CELL_MM;
    walls_current_cell.right = tof.dist_right_mm < WALL_PRESENT_CURRENT_CELL_MM;
    walls_current_cell.front = tof.dist_front_mm < WALL_PRESENT_CURRENT_CELL_MM;
}

void tof_check_walls_next_cell() {
    using namespace Micromouse;
    walls_next_cell.left = tof.dist_diag_left_mm < WALL_PRESENT_NEXT_CELL_DIAG_MM;
    walls_next_cell.right = tof.dist_diag_right_mm < WALL_PRESENT_NEXT_CELL_DIAG_MM;
    walls_next_cell.front = tof.dist_front_mm < WALL_PRESENT_NEXT_CELL_FRONT_MM;
}



// bool tof_read_if_ready() {
//     using namespace Micromouse;
//     bool got_any_new_data = false;

//     got_any_new_data |= read_one_sensor(tof_left, TOF_CH_LEFT, tof.dist_left_mm);
//     got_any_new_data |= read_one_sensor(tof_right, TOF_CH_RIGHT, tof.dist_right_mm);
//     got_any_new_data |= read_one_sensor(tof_front, TOF_CH_FRONT, tof.dist_front_mm);
//     got_any_new_data |= read_one_sensor(tof_diag_left, TOF_CH_DIAG_L, tof.dist_diag_left_mm);
//     got_any_new_data |= read_one_sensor(tof_diag_right, TOF_CH_DIAG_R, tof.dist_diag_right_mm);
    
//     tof.data_refreshed = got_any_new_data;
//     // mux_disable();
//     return got_any_new_data;
// }


// // read one measurement (mm) from the currently selected channel
// // returns -1 if invalid
// int16_t read_one_sensor(VL53L4CD &sensor, uint8_t channel, float distance_mm) {
//     uint8_t ready = 0;
//     VL53L4CD_Result_t result;
    
//     // wait for data ready (with timeout)
//     unsigned long t0 = millis();
//     while (!ready && (millis() - t0 < 200)) {
//         sensor.VL53L4CD_CheckForDataReady(&ready);
//     }
//     if (!ready) return -1;
    
//     sensor.VL53L4CD_GetResult(&result);
//     sensor.VL53L4CD_ClearInterrupt();

//     // range_status == 0 usually means valid reading
//     if (result.range_status != 0) { return -1; }
//     else {  distance_mm = (int16_t)result.distance_mm;  }
    
//     return distance_mm;
// }
