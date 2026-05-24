/**
 * @file tof.cpp
 * @brief reads VL53L4CD TOF sensors through PCA9548 I2C mux
 */

#include "tof.h"
#include "globals.h"
#include "config.h"

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>

#define DEV_I2C Wire

// ----- TOF mux channels -----
static constexpr uint8_t TOF_CH_LEFT       = 0;
static constexpr uint8_t TOF_CH_RIGHT      = 1;
static constexpr uint8_t TOF_CH_FRONT      = 2;
static constexpr uint8_t TOF_CH_FRONT_LEFT = 3;
static constexpr uint8_t TOF_CH_FRONT_RIGHT = 4;

// ----- sensor objects -----
static VL53L4CD tof_left(&DEV_I2C, A1);
static VL53L4CD tof_right(&DEV_I2C, A1);
static VL53L4CD tof_front(&DEV_I2C, A1);
static VL53L4CD tof_front_left(&DEV_I2C, A1);
static VL53L4CD tof_front_right(&DEV_I2C, A1);

static void mux_select(uint8_t ch) {
    if (ch > 7) {
        return;
    }

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
    // Smaller = faster but noisier.
    sensor.VL53L4CD_SetRangeTiming(10, 0);

    sensor.VL53L4CD_StartRanging();

    delay(10);
}

static bool read_one_sensor(VL53L4CD &sensor, uint8_t channel, float &distance_m) {
    mux_select(channel);

    uint8_t new_data_ready = 0;
    uint8_t status = sensor.VL53L4CD_CheckForDataReady(&new_data_ready);

    if (status || !new_data_ready) {
        return false;
    }

    VL53L4CD_Result_t result;

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

    Micromouse::tof.wall_left = false;
    Micromouse::tof.wall_right = false;
    Micromouse::tof.wall_front = false;
    Micromouse::tof.data_refreshed = false;
}

bool tof_read_if_ready() {
    using namespace Micromouse;

    bool got_any_new_data = false;

    got_any_new_data |= read_one_sensor(tof_left, TOF_CH_LEFT, tof.dist_left_m);
    got_any_new_data |= read_one_sensor(tof_right, TOF_CH_RIGHT, tof.dist_right_m);
    got_any_new_data |= read_one_sensor(tof_front, TOF_CH_FRONT, tof.dist_front_m);

    // These are read for now, but not stored yet because globals.h does not currently
    // have fields for angled sensors.
    static float dist_front_left_m = TOF_MAX_RANGE_M;
    static float dist_front_right_m = TOF_MAX_RANGE_M;

    got_any_new_data |= read_one_sensor(tof_front_left, TOF_CH_FRONT_LEFT, dist_front_left_m);
    got_any_new_data |= read_one_sensor(tof_front_right, TOF_CH_FRONT_RIGHT, dist_front_right_m);

    tof.wall_left = tof.dist_left_m < WALL_PRESENT_M;
    tof.wall_right = tof.dist_right_m < WALL_PRESENT_M;
    tof.wall_front = tof.dist_front_m < WALL_PRESENT_M;

    tof.data_refreshed = got_any_new_data;

    mux_disable();

    return got_any_new_data;
}
