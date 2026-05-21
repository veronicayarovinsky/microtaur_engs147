// vl6180x_example.cpp
// Adafruit example code for vl6180x TOF sensor

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_VL6180X.h>

Adafruit_VL6180X vl = Adafruit_VL6180X();

void setup() {
    Wire.begin();
    SerialUSB.begin(115200);

    // wait for serial port to open on native usb devices
    while (!SerialUSB) {
        delay(1);
    }
  
    SerialUSB.println("Adafruit VL6180x test!");
    while (! vl.begin()) {
        SerialUSB.println("Failed to find sensor");
        delay(2000);
    }
//   if (! vl.begin()) {
//     SerialUSB.println("Failed to find sensor");
//     while (1);
//   }
    SerialUSB.println("Sensor found!");
}

void loop() {
//   float lux = vl.readLux(VL6180X_ALS_GAIN_5);
//   Serial.print("Lux: "); Serial.println(lux);
    uint8_t range = vl.readRange();
    uint8_t status = vl.readRangeStatus();

    if (status == VL6180X_ERROR_NONE) {
        // SerialUSB.print("Range: ");
        SerialUSB.println(range);
    }

    // Some error occurred, print it out!
    
    if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
        SerialUSB.println("System error");
    }
    else if (status == VL6180X_ERROR_ECEFAIL) {
        SerialUSB.println("ECE failure");
    }
    else if (status == VL6180X_ERROR_NOCONVERGE) {
        SerialUSB.println("No convergence");
    }
    else if (status == VL6180X_ERROR_RANGEIGNORE) {
        SerialUSB.println("Ignoring range");
    }
    else if (status == VL6180X_ERROR_SNR) {
        Serial.println("Signal/Noise error");
    }
    else if (status == VL6180X_ERROR_RAWUFLOW) {
        SerialUSB.println("Raw reading underflow");
    }
    else if (status == VL6180X_ERROR_RAWOFLOW) {
        SerialUSB.println("Raw reading overflow");
    }
    else if (status == VL6180X_ERROR_RANGEUFLOW) {
        SerialUSB.println("Range reading underflow");
    }
    else if (status == VL6180X_ERROR_RANGEOFLOW) {
        SerialUSB.println("Range reading overflow");
    }
    delay(1000);    // 1 second pause
}
