#include "flood_fill.h"
#include <Arduino.h>

static int flood[MAZE_SIZE][MAZE_SIZE];

static bool wall_north[MAZE_SIZE][MAZE_SIZE];
static bool wall_east[MAZE_SIZE][MAZE_SIZE];
static bool wall_south[MAZE_SIZE][MAZE_SIZE];
static bool wall_west[MAZE_SIZE][MAZE_SIZE];

static bool in_bounds(int x, int y) {
    return x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE;
}

static bool has_wall(int x, int y, Direction dir) {
    if (dir == NORTH) return wall_north[x][y];
    if (dir == EAST)  return wall_east[x][y];
    if (dir == SOUTH) return wall_south[x][y];
    if (dir == WEST)  return wall_west[x][y];

    return true;
}

static int neighbor_x(int x, Direction dir) {
    if (dir == EAST) return x + 1;
    if (dir == WEST) return x - 1;
    return x;
}

static int neighbor_y(int y, Direction dir) {
    if (dir == NORTH) return y + 1;
    if (dir == SOUTH) return y - 1;
    return y;
}

static Direction left_of(Direction dir) {
    return (Direction)((dir + 3) % 4);
}

static Direction right_of(Direction dir) {
    return (Direction)((dir + 1) % 4);
}

void maze_set_wall(int x, int y, Direction dir, bool exists) {
    if (!in_bounds(x, y)) return;

    if (dir == NORTH) {
        wall_north[x][y] = exists;
        if (in_bounds(x, y + 1)) wall_south[x][y + 1] = exists;
    }

    if (dir == EAST) {
        wall_east[x][y] = exists;
        if (in_bounds(x + 1, y)) wall_west[x + 1][y] = exists;
    }

    if (dir == SOUTH) {
        wall_south[x][y] = exists;
        if (in_bounds(x, y - 1)) wall_north[x][y - 1] = exists;
    }

    if (dir == WEST) {
        wall_west[x][y] = exists;
        if (in_bounds(x - 1, y)) wall_east[x - 1][y] = exists;
    }
}

static void update_walls_from_tof(
    int x,
    int y,
    Direction heading,
    const Micromouse::WallsCurrentCell& walls
) {
    if (walls.front != -1) {
        maze_set_wall(x, y, heading, walls.front == 1);
    }

    if (walls.left != -1) {
        maze_set_wall(x, y, left_of(heading), walls.left == 1);
    }

    if (walls.right != -1) {
        maze_set_wall(x, y, right_of(heading), walls.right == 1);
    }
}

void flood_init() {
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            flood[x][y] = 255;

            wall_north[x][y] = false;
            wall_east[x][y] = false;
            wall_south[x][y] = false;
            wall_west[x][y] = false;
        }
    }

    for (int i = 0; i < MAZE_SIZE; i++) {
        wall_south[i][0] = true;
        wall_north[i][MAZE_SIZE - 1] = true;
        wall_west[0][i] = true;
        wall_east[MAZE_SIZE - 1][i] = true;
    }
}

void flood_fill() {
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            flood[x][y] = 255;
        }
    }

    int goal_x[4] = {7, 7, 8, 8};
    int goal_y[4] = {7, 8, 7, 8};

    int queue_x[MAZE_SIZE * MAZE_SIZE];
    int queue_y[MAZE_SIZE * MAZE_SIZE];

    int head = 0;
    int tail = 0;

    for (int i = 0; i < 4; i++) {
        int gx = goal_x[i];
        int gy = goal_y[i];

        flood[gx][gy] = 0;

        queue_x[tail] = gx;
        queue_y[tail] = gy;
        tail++;
    }

    while (head < tail) {
        int x = queue_x[head];
        int y = queue_y[head];
        head++;

        int current_value = flood[x][y];

        for (int d = 0; d < 4; d++) {
            Direction dir = (Direction)d;

            if (has_wall(x, y, dir)) continue;

            int nx = neighbor_x(x, dir);
            int ny = neighbor_y(y, dir);

            if (!in_bounds(nx, ny)) continue;

            if (flood[nx][ny] > current_value + 1) {
                flood[nx][ny] = current_value + 1;

                queue_x[tail] = nx;
                queue_y[tail] = ny;
                tail++;
            }
        }
    }
}

Direction flood_get_best_direction(int x, int y) {
    Direction best_dir = NORTH;
    int best_value = 255;

    for (int d = 0; d < 4; d++) {
        Direction dir = (Direction)d;

        if (has_wall(x, y, dir)) continue;

        int nx = neighbor_x(x, dir);
        int ny = neighbor_y(y, dir);

        if (!in_bounds(nx, ny)) continue;

        if (flood[nx][ny] < best_value) {
            best_value = flood[nx][ny];
            best_dir = dir;
        }
    }

    return best_dir;
}

FloodOutput flood_fill_step(
    int x,
    int y,
    int a_global,
    const Micromouse::WallsCurrentCell& walls
) {
    Direction heading = (Direction)a_global;

    update_walls_from_tof(x, y, heading, walls);

    flood_fill();

    Direction best_dir = flood_get_best_direction(x, y);

    FloodOutput output;
    output.want_dir = best_dir;
    output.x_want = neighbor_x(x, best_dir);
    output.y_want = neighbor_y(y, best_dir);
    output.a_want = (int)best_dir;

    return output;
}