// src/display.cpp

#include "config.h"
#include "globals.h"
#include "display.h"
// #include "tof.h"

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_W 128
#define SCREEN_H 64

#define S SerialUSB

static Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &SPI, PIN_OLED_DC, PIN_OLED_RST, PIN_OLED_CS);

void display_init() {
    if (!oled.begin(SSD1306_SWITCHCAPVCC)) {
        S.println("OLED init failed");
        return;
    }
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setCursor(0, 0);
    oled.println("welcome to");
    oled.println("the club");
    oled.println("micromice :)");
    oled.display();
}

// printing values that have units mm
void display_tof_dist() {
    using namespace Micromouse;
    oled.print("        F  "); oled.println(tof.dist_front_mm, 1);
    oled.print("DL "); oled.print(tof.dist_diag_left_mm, 1); oled.print("    DR "); oled.println(tof.dist_diag_right_mm, 1);
    oled.print("L  "); oled.print(tof.dist_left_mm, 1); oled.print("    R  "); oled.println(tof.dist_right_mm, 1);
}

void display_walls_current_cell() {
    // oled.println("Walls Current Cell");
    // oled.println("L F R");
    oled.print("wallsCurr: LFR = ");
    oled.print(Micromouse::walls_current_cell.left);
    oled.print(Micromouse::walls_current_cell.front);
    oled.println(Micromouse::walls_current_cell.right);
}

void display_walls_next_cell() {
    oled.print("wallsNext: LFR = ");
    // oled.println("L F R");
    oled.print(Micromouse::walls_next_cell.left);
    oled.print(Micromouse::walls_next_cell.front);
    oled.println(Micromouse::walls_next_cell.right);
}

void display_update() {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0, 0);

    display_tof_dist();
    display_walls_current_cell();
    display_walls_next_cell();

    oled.display();  // sends buffer to screen over SPI
}

void display_print() {
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setCursor(0, 0);
    oled.println("maze solved!");
    oled.display();
}

void display_direction(const char* dir)
{
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0,0);

    display_tof_dist();
    display_walls_current_cell();
    display_walls_next_cell();

    oled.print("Move: ");
    oled.println(dir);

    oled.display();
}