#include "tof.h"
#include "globals.h"
#include "config.h"
#include <Wire.h>
#include <vl53l4cd_class.h>

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


void tof_init() {
    for (uint8_t ch = 0; ch < 3; ch++) {
        mux_select(ch);
        
        mux_disable();
    }
}



bool tof_read_if_ready() {
    using namespace Micromouse;
    
    tof.dist_left_m  = 0.09f;
    tof.dist_right_m = 0.09f;
    tof.dist_front_m = 0.20f;
    tof.wall_left    = tof.dist_left_m  < WALL_PRESENT_M;
    tof.wall_right   = tof.dist_right_m < WALL_PRESENT_M;
    tof.wall_front   = tof.dist_front_m < WALL_PRESENT_M;
    tof.data_fresh   = true;
    return true;
}
