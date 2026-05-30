// i2c_scanner_example.cpp
// source: https://learn.adafruit.com/scanning-i2c-addresses/arduino
//
// Modified from https://playground.arduino.cc/Main/I2cScanner/
// --------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BusIO_Register.h>

// Set I2C bus to use: Wire, Wire1, etc.
#define WIRE Wire1
#define S SerialUSB

void setup() {
    S.begin(115200);
    while(!S){}
    WIRE.begin();
    WIRE.setClock(100000);
    S.println("\nI2C Scanner");
}


void loop() {
    // byte error, address;
    int nDevices;

    S.println("Scanning...");

    nDevices = 0;
    for(byte address = 1; address < 127; address++ ) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        WIRE.beginTransmission(address);
        byte error = WIRE.endTransmission();

        if (error == 0) {
        S.print("I2C device found at address 0x");
        if (address<16)
            S.print("0");
        S.print(address,HEX);
        S.println("  !");

        nDevices++;
        }
        else if (error==4) {
            S.print("Unknown error at address 0x");
            if (address<16) {
                S.print("0");
            }
            S.println(address,HEX);
        }
    }
    if (nDevices == 0) {
        S.println("No I2C devices found\n");
    }   
    else {
        S.println("done\n");
    }

    delay(5000);           // wait 5 seconds for next scan
    // while(1);
}
