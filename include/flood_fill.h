#ifndef FLOOD_FILL_H
#define FLOOD_FILL_H

#include "config.h"

enum Direction {
    NORTH,
    EAST,
    SOUTH,
    WEST
};

void flood_init();
void flood_fill();

Direction flood_get_best_direction(int x, int y);

void maze_set_wall(int x, int y, Direction dir, bool exists);

#endif