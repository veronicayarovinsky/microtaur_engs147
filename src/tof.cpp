#include "tof.h"
#include "globals.h"
#include "config.h"
#include <Wire.h>
#include <vl53l4cd_class.h>

VL53L4CD sensor(&Wire, -1);

// select a single channel on the PCA9548 mux
static void mux_select(uint8_t ch) {
    Wire.beginTransmission(I2C_ADDR_PCA9548);
    Wire.write(1 << ch);
    Wire.endTransmission();
}
static void mux_disable() {
    Wire.beginTransmission(I2C_ADDR_PCA9548);
    Wire.write(0x00);
    Wire.endTransmission();
}

// initialize the sensor currently selected on the mux
void init_one_sensor() {
    sensor.InitSensor(0x29);
    sensor.VL53L4CD_SetRangeTiming(TOF_RANGE_TIMING_BUDGET_MS, 0);  // 50 ms timing budget
    sensor.VL53L4CD_StartRanging();
}


void tof_init() {
    uint8_t channels[NUM_TOF_SENSORS] = {TOF_CH_LEFT, TOF_CH_DIAG_L,
                        TOF_CH_FRONT, TOF_CH_DIAG_R, TOF_CH_RIGHT};
    for (int i = 0; i < NUM_TOF_SENSORS; i++) {
        mux_select(channels[i]);
        delay(50);
        init_one_sensor();
        delay(50);
        mux_disable();
    }
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

bool tof_read_if_ready() {
    using namespace Micromouse;
    mux_select(TOF_CH_LEFT);
    tof.dist_left_m  = readOneSensor();
    mux_select(TOF_CH_DIAG_L);
    tof.dist_diagL_m = readOneSensor();
    mux_select(TOF_CH_RIGHT);
    tof.dist_right_m = readOneSensor();
    mux_select(TOF_CH_DIAG_R);
    tof.dist_diagR_m = readOneSensor();
    mux_select(TOF_CH_FRONT);
    tof.dist_front_m = readOneSensor();

    tof.wall_left    = tof.dist_left_m  < WALL_PRESENT_M;
    tof.wall_right   = tof.dist_right_m < WALL_PRESENT_M;
    tof.wall_front   = tof.dist_front_m < WALL_PRESENT_M;
    tof.data_refreshed   = true;
    return true;
}
