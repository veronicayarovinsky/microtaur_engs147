// BNO055 Calibration Utility - Arduino Due / PlatformIO
// Purpose: walk the sensor through its calibration routine and print the
//          resulting offsets so they can be hardcoded into the test/runtime code.
//
// Calibration procedure (per BNO055 datasheet section 3.11):
//   - GYRO: place sensor stationary for a few seconds.
//   - ACCEL: hold sensor in 6 different stable orientations (~+/-X, +/-Y, +/-Z up),
//            ~2-3 seconds each, with brief motion between them.
//   - MAG:  move sensor in a figure-8 pattern in the air.
//   - SYS:  reaches 3 once all three above are calibrated and the fusion
//           algorithm has settled.
//
// Each calibration status is reported on a 0-3 scale (3 = fully calibrated).
// Once SYS == 3, the script prints the offset registers; copy these values
// into bno055_heading_test.cpp to skip recalibration on every power-up.

/*
void Adafruit_BNO055::getCalibration(uint8_t *system, uint8_t *gyro, uint8_t *accel, uint8_t *mag)
Gets current calibration state. Each value should be a uint8_t pointer and it will be set to 0 if not calibrated and 3 if fully calibrated. See section 34.3.54
Parameters:
sys – Current system calibration status, depends on status of all sensors, read-only
gyro – Current calibration status of Gyroscope, read-only
accel – Current calibration status of Accelerometer, read-only
mag – Current calibration status of Magnetometer, read-only
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// config constants
#define BAUD_RATE       115200
#define BNO055_ADDR     0x28        // ADR pin = HIGH per schematic -> 0x29; pin LOW -> 0x28.
                                    // Proposal shows ADR HI / addr 0x28 -- using that.
#define SAMPLE_PERIOD   100         // ms between status prints
#define SENSOR_ID       55

// pass (sensor_id, i2c_addr, wire). Default Wire is I2C bus 0 (SDA/SCL pins on Due).
Adafruit_BNO055 bno = Adafruit_BNO055(SENSOR_ID, BNO055_ADDR, &Wire);

bool offsets_saved = false;

void printOffsets(const adafruit_bno055_offsets_t &offsets) {
    SerialUSB.println();
    SerialUSB.println("======== CALIBRATION OFFSETS ========");
    SerialUSB.print("Accel offsets (X,Y,Z): ");
    SerialUSB.print(offsets.accel_offset_x); SerialUSB.print(", ");
    SerialUSB.print(offsets.accel_offset_y); SerialUSB.print(", ");
    SerialUSB.println(offsets.accel_offset_z);

    SerialUSB.print("Gyro  offsets (X,Y,Z): ");
    SerialUSB.print(offsets.gyro_offset_x); SerialUSB.print(", ");
    SerialUSB.print(offsets.gyro_offset_y); SerialUSB.print(", ");
    SerialUSB.println(offsets.gyro_offset_z);

    SerialUSB.print("Mag   offsets (X,Y,Z): ");
    SerialUSB.print(offsets.mag_offset_x); SerialUSB.print(", ");
    SerialUSB.print(offsets.mag_offset_y); SerialUSB.print(", ");
    SerialUSB.println(offsets.mag_offset_z);

    SerialUSB.print("Accel radius: "); SerialUSB.println(offsets.accel_radius);
    SerialUSB.print("Mag   radius: "); SerialUSB.println(offsets.mag_radius);
    SerialUSB.println("=====================================");

    // emit as a copy-pasteable C struct literal
    SerialUSB.println();
    SerialUSB.println("// Paste into bno055_heading_test.cpp:");
    SerialUSB.println("adafruit_bno055_offsets_t calibrationData = {");
    SerialUSB.print("    "); SerialUSB.print(offsets.accel_offset_x); SerialUSB.print(", ");
    SerialUSB.print(offsets.accel_offset_y); SerialUSB.print(", ");
    SerialUSB.print(offsets.accel_offset_z); SerialUSB.println(",  // accel x,y,z");
    SerialUSB.print("    "); SerialUSB.print(offsets.mag_offset_x); SerialUSB.print(", ");
    SerialUSB.print(offsets.mag_offset_y); SerialUSB.print(", ");
    SerialUSB.print(offsets.mag_offset_z); SerialUSB.println(",  // mag   x,y,z");
    SerialUSB.print("    "); SerialUSB.print(offsets.gyro_offset_x); SerialUSB.print(", ");
    SerialUSB.print(offsets.gyro_offset_y); SerialUSB.print(", ");
    SerialUSB.print(offsets.gyro_offset_z); SerialUSB.println(",  // gyro  x,y,z");
    SerialUSB.print("    "); SerialUSB.print(offsets.accel_radius); SerialUSB.println(",  // accel radius");
    SerialUSB.print("    "); SerialUSB.print(offsets.mag_radius); SerialUSB.println("   // mag   radius");
    SerialUSB.println("};");
}

void setup() {
    SerialUSB.begin(BAUD_RATE);
    while (!SerialUSB) { delay(10); }   // wait for USB CDC enumeration on Due

    SerialUSB.println("BNO055 Calibration Utility");
    SerialUSB.println("--------------------------");

    if (!bno.begin()) {
        SerialUSB.println("ERROR: BNO055 not detected. Check wiring and I2C address.");
        while (1) { delay(1000); }
    }

    delay(1000);
    bno.setExtCrystalUse(true);   // use the external 32 kHz crystal on the Adafruit breakout

    SerialUSB.println("Sensor detected. Begin calibration:");
    SerialUSB.println("  1. Keep stationary on a flat surface (gyro).");
    SerialUSB.println("  2. Tilt through 6 orthogonal orientations, pause at each (accel).");
    SerialUSB.println("  3. Move in a figure-8 pattern through the air (mag).");
    SerialUSB.println();
    SerialUSB.println("Status legend: 0 = uncalibrated, 3 = fully calibrated");
    SerialUSB.println("time_ms,sys,gyro,accel,mag");
}

void loop() {
    uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    SerialUSB.print(millis());
    SerialUSB.print(",");
    SerialUSB.print(sys); SerialUSB.print(",");
    SerialUSB.print(gyro); SerialUSB.print(",");
    SerialUSB.print(accel); SerialUSB.print(",");
    SerialUSB.println(mag);

    // once fully calibrated, dump offsets once and stop
    if (!offsets_saved && bno.isFullyCalibrated()) {
        adafruit_bno055_offsets_t offsets;
        if (bno.getSensorOffsets(offsets)) {
            printOffsets(offsets);
            offsets_saved = true;
            SerialUSB.println();
            SerialUSB.println("Calibration complete. Halting.");
            while (1) { delay(1000); }
        }
    }

    delay(SAMPLE_PERIOD);
}
