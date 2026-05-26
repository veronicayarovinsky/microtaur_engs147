#include "imu.h"
#include "globals.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_BNO055.h>

adafruit_bno055_offsets_t calibrationData = {
    -50, 37, -33,       // accel x,y,z
    85, -363, -559,     // mag   x,y,z
    -1, -1, 0,          // gyro  x,y,z
    1000,               // accel radius
    686                 // mag   radius
};

Adafruit_BNO055 bno = Adafruit_BNO055(SENSOR_ID, I2C_ADDR_BNO055, &Wire);

void imu_init() {
    Wire.begin();
    bno.begin();
    bno.setMode(OPERATION_MODE_NDOF);  // or OPERATION_MODE_IMUPLUS if motors disturb mag
    delay(500);
    bno.setExtCrystalUse(true);
    bno.setSensorOffsets(calibrationData);
    Micromouse::imu.ready = false;
}

void imu_update() {
    using namespace Micromouse;
    sensors_event_t euler_event, gyro_event;
    
    bno.getEvent(&euler_event, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&gyro_event,  Adafruit_BNO055::VECTOR_GYROSCOPE);
    
    // Adafruit BNO055 maps Euler X -> heading/yaw (0->360 deg)
    Micromouse::imu.heading_rad     = euler_event.orientation.x * PI / 180.0f;
    Micromouse::imu.omega_imu_rad_s = gyro_event.gyro.z;
    Micromouse::imu.ready           = true;
}

void get_imu_calibration_status() {
    using namespace Micromouse;
    uint8_t s, g, a, m;
    bno.getCalibration(&s, &g, &a, &m);

    Micromouse::imu_calibration.sys_cal = s;
    Micromouse::imu_calibration.gyr_cal = g;
    Micromouse::imu_calibration.acc_cal = a;
    Micromouse::imu_calibration.mag_cal = m;
}
