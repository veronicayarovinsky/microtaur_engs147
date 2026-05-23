// src/display.cpp
#include "../include/display.h"
#include "../include/globals.h"
#include "../include/config.h"

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_W 128
#define SCREEN_H 64

static Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &SPI, PIN_OLED_DC, PIN_OLED_RST, PIN_OLED_CS);

void display_init() {
    if (!oled.begin(SSD1306_SWITCHCAPVCC)) {
        SerialUSB.println("OLED init failed");
        return;
    }
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.display();
}

void display_update() {
    oled.clearDisplay();
    oled.setCursor(0, 0);

    oled.print("Micromouse project!!");

    oled.display();  // sends buffer to screen over SPI
}
