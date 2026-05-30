/**
 * @file display.h
 * @brief 
 * 
 * datasheet link: https://www.buydisplay.com/download/manual/ER-OLEDM0.96-1_Series_Datasheet.pdf
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>

void display_init();
void display_update();
void display_print();
void display_direction(const char* dir);

#endif
