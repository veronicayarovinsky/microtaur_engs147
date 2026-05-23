#ifndef FSM_H
#define FSM_H

#include <Arduino.h>

void drive_init();
void drive_stop();

bool drive_forward(float distance_m, float speed_m_s);
bool drive_turn(float delta_heading_rad);

#endif
