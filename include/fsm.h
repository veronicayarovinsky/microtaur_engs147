/**
 * @file fsm.h
 * @brief Finite State Machine
 *        writes --> Micromouse::cell, walls, state
 *        calls drive_forward(), drive_turn(), drive_stop() from drive.h
 */
#ifndef FSM_H
#define FSM_H

#include <Arduino.h>

void fsm_init();
void fsm_update();

#endif