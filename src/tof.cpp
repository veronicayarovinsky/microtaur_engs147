#include "tof.h"
#include "globals.h"
#include "config.h"
#include <Wire.h>
#include <vl53l4cd_class.h>

#define DEV_I2C Wire

// TOF mux channels
static constexpr uint8_t TOF_CH_LEFT = 0;
static constexpr uint8_t TOF_CH_RIGHT = 1;
static constexpr uint8_t TOF_CH_FRONT = 2;
static constexpr uint8_t TOF_CH_FRONT_LEFT = 3;
static constexpr uint8_t TOF_CH_FRONT_RIGHT = 4;

// sensor objects
static VL53L4CD tof_left(&DEV_I2C, A1);
static VL53L4CD tof_right(&DEV_I2C, A1);
static VL53L4CD tof_front(&DEV_I2C, A1);
static VL53L4CD tof_front_left(&DEV_I2C, A1);
static VL53L4CD tof_front_right(&DEV_I2C, A1);

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

static void init_one_sensor(VL53L4CD &sensor, uint8_t channel) {
    mux_select(channel);

    sensor.begin();
    sensor.VL53L4CD_Off();
    sensor.InitSensor();

    // timing budget in ms
    // smaller = faster but noisier
    sensor.VL53L4CD_SetRangeTiming(10, 0);

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

    // range_status == 0 usually means valid reading
    if (result.range_status == 0) {
        distance_m = result.distance_mm / 1000.0f;
    } else {
        distance_m = TOF_MAX_RANGE_M;
    }

    return true;
}

void tof_init() {
    Wire.begin();

    init_one_sensor(tof_left, TOF_CH_LEFT);
    init_one_sensor(tof_right, TOF_CH_RIGHT);
    init_one_sensor(tof_front, TOF_CH_FRONT);
    init_one_sensor(tof_front_left, TOF_CH_FRONT_LEFT);
    init_one_sensor(tof_front_right, TOF_CH_FRONT_RIGHT);

    mux_disable();

    Micromouse::tof.dist_left_m = TOF_MAX_RANGE_M;
    Micromouse::tof.dist_right_m = TOF_MAX_RANGE_M;
    Micromouse::tof.dist_front_m = TOF_MAX_RANGE_M;
    Micromouse::tof.dist_front_left_m = TOF_MAX_RANGE_M;
    Micromouse::tof.dist_front_right_m = TOF_MAX_RANGE_M;

    Micromouse::tof.wall_left = false;
    Micromouse::tof.wall_right = false;
    Micromouse::tof.wall_front = false;
    Micromouse::tof.wall_front_left = false;
    Micromouse::tof.wall_front_right = false;

    Micromouse::tof.data_refreshed = false;
}

bool tof_read_if_ready() {
    using namespace Micromouse;

    bool got_any_new_data = false;

    got_any_new_data |= read_one_sensor(tof_left, TOF_CH_LEFT, tof.dist_left_m);
    got_any_new_data |= read_one_sensor(tof_right, TOF_CH_RIGHT, tof.dist_right_m);
    got_any_new_data |= read_one_sensor(tof_front, TOF_CH_FRONT, tof.dist_front_m);
    got_any_new_data |= read_one_sensor(tof_front_left, TOF_CH_FRONT_LEFT, tof.dist_front_left_m);
    got_any_new_data |= read_one_sensor(tof_front_right, TOF_CH_FRONT_RIGHT, tof.dist_front_right_m);

    tof.wall_left = tof.dist_left_m < WALL_PRESENT_M;
    tof.wall_right = tof.dist_right_m < WALL_PRESENT_M;
    tof.wall_front = tof.dist_front_m < WALL_PRESENT_M;
    tof.wall_front_left = tof.dist_front_left_m < WALL_PRESENT_M;
    tof.wall_front_right = tof.dist_front_right_m < WALL_PRESENT_M;

    tof.data_refreshed = got_any_new_data;

    mux_disable();

    return got_any_new_data;
}
