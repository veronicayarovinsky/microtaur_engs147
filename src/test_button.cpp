// Pushbutton
// active LOW aka LOW when pressed, HIGH when released
// example code for how to use:
/*  static bool last_btn = HIGH;
    bool btn = digitalRead(PIN_BUTTON);
    if (last_btn == HIGH && btn == LOW) {
        do_something();
        last_btn = btn;
    }                                        */
// constexpr int PIN_BUTTON = 33;

#include <Arduino.h>
#include "config.h"
#include "globals.h"

static bool last_btn = HIGH; 

void setup() {
    Serial.begin(BAUD_RATE);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    
    Serial.println("Button Test Initialized");
}

void loop() {
    bool btn = digitalRead(PIN_BUTTON);

    if (last_btn == HIGH && btn == LOW) {
        Serial.println("Button Pressed");
    } 
    else if (last_btn == LOW && btn == HIGH) {
        Serial.println("Button Released");
    }

    last_btn = btn;

    delay(50);
}