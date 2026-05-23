#include "imu.h"
#include "globals.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_BNO055.h>

static Adafruit_BNO055 bno(55, I2C_ADDR_BNO055);

void imu_init() {
    Wire.begin();
    bno.begin();
    bno.setMode(OPERATION_MODE_NDOF);  // or OPERATION_MODE_IMUPLUS if motors disturb mag
    delay(500);
    Micromouse::imu.ready = false;
}

void imu_update() {
    using namespace Micromouse;
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu.heading_rad     = euler.x() * PI / 180.0f;
    imu.omega_imu_rad_s = gyro.z();
    imu.ready           = true;
}
