#ifndef FLOOD_FILL_H
#define FLOOD_FILL_H

#include "config.h"
#include "globals.h"

enum Direction {
    NORTH = 0,
    EAST  = 1,
    SOUTH = 2,
    WEST  = 3
};

struct FloodOutput {
    Direction want_dir;   // direction flood fill wants robot to face
    int x_want;           // next cell x
    int y_want;           // next cell y
    int a_want;           // same as want_dir, but as an int
};

void flood_init();

FloodOutput flood_fill_step(
    int x,
    int y,
    int a_global,
    const Micromouse::WallsCurrentCell& walls
);

void flood_fill();

Direction flood_get_best_direction(int x, int y);

void maze_set_wall(int x, int y, Direction dir, bool exists);

#endif