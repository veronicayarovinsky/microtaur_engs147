#include <Arduino.h>

#define LED_PIN 13 // replace xx with Due digital pin number for LED

void setup() {
    // initialize the serial port using a baud rate of 115,200 bits per second
    SerialUSB.begin(9600);
    SerialUSB.println("Hello, world!");
    
    // configure the digital pin attached to the LED as an output
    pinMode(LED_PIN,1);
}

void loop() {
    // declare and initialize variables
    uint8_t led_state = 0;
    uint32_t i = 0;
  
    // infinite loop
    while(1){
  
      // print integer to serial port
      SerialUSB.println(i);
      ++i;
  
      // blink LED
      digitalWrite(LED_PIN,led_state);
      led_state = ~led_state;
      
      // wait 0.5 sec
      // NOTE: this is *NOT* a good way to enforce loop timing!
      delay(500);
    }
}
  


