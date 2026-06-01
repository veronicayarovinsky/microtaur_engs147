#include "flood_fill.h"
#include <Arduino.h>
#include <stdlib.h>

static int flood[MAZE_SIZE][MAZE_SIZE];

static bool wall_north[MAZE_SIZE][MAZE_SIZE];
static bool wall_east[MAZE_SIZE][MAZE_SIZE];
static bool wall_south[MAZE_SIZE][MAZE_SIZE];
static bool wall_west[MAZE_SIZE][MAZE_SIZE];

static bool in_bounds(int x, int y) {
    return x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE;
}

static bool is_goal(int x, int y) {
    return (x == 7 || x == 8) && (y == 7 || y == 8);
}

static Direction opposite(Direction dir) {
    return (Direction)((dir + 2) % 4);
}

static Direction left_of(Direction dir) {
    return (Direction)((dir + 3) % 4);
}

static Direction right_of(Direction dir) {
    return (Direction)((dir + 1) % 4);
}

// GLOBAL maze direction movement.
// NORTH/SOUTH/EAST/WEST are fixed to the maze, not the robot.
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

static bool has_wall(int x, int y, Direction dir) {
    if (!in_bounds(x, y)) return true;

    if (dir == NORTH) return wall_north[x][y];
    if (dir == EAST)  return wall_east[x][y];
    if (dir == SOUTH) return wall_south[x][y];
    if (dir == WEST)  return wall_west[x][y];

    return true;
}

void maze_set_wall(int x, int y, Direction dir, bool exists) {
    if (!in_bounds(x, y)) return;

    if (dir == NORTH) {
        wall_north[x][y] = exists;
        if (in_bounds(x, y + 1)) wall_south[x][y + 1] = exists;
    }

    else if (dir == EAST) {
        wall_east[x][y] = exists;
        if (in_bounds(x + 1, y)) wall_west[x + 1][y] = exists;
    }

    else if (dir == SOUTH) {
        wall_south[x][y] = exists;
        if (in_bounds(x, y - 1)) wall_north[x][y - 1] = exists;
    }

    else if (dir == WEST) {
        wall_west[x][y] = exists;
        if (in_bounds(x - 1, y)) wall_east[x - 1][y] = exists;
    }
}

// Converts ROBOT-relative sensor readings into GLOBAL maze walls.
static void update_walls_from_tof(
    int x,
    int y,
    Direction robot_heading,
    const Micromouse::WallsCurrentCell& walls
) {
    if (walls.front != -1) {
        maze_set_wall(x, y, robot_heading, walls.front == 1);
    }

    if (walls.left != -1) {
        maze_set_wall(x, y, left_of(robot_heading), walls.left == 1);
    }

    if (walls.right != -1) {
        maze_set_wall(x, y, right_of(robot_heading), walls.right == 1);
    }
}

// Manhattan distance to nearest center cell.
static void init_manhattan_distances() {
    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            if (is_goal(x, y)) {
                flood[x][y] = 0;
            } else {
                int dx = max(0, max(7 - x, x - 8));
                int dy = max(0, max(7 - y, y - 8));
                flood[x][y] = dx + dy;
            }
        }
    }
}

void flood_init() {
    Serial.println("FLOOD_INIT CALLED");

    for (int x = 0; x < MAZE_SIZE; x++) {
        for (int y = 0; y < MAZE_SIZE; y++) {
            wall_north[x][y] = false;
            wall_east[x][y] = false;
            wall_south[x][y] = false;
            wall_west[x][y] = false;
        }
    }

    // Boundary walls.
    for (int i = 0; i < MAZE_SIZE; i++) {
        wall_south[i][0] = true;
        wall_north[i][MAZE_SIZE - 1] = true;
        wall_west[0][i] = true;
        wall_east[MAZE_SIZE - 1][i] = true;
    }

    init_manhattan_distances();

    Serial.print("MAZE_SIZE = ");
    Serial.println(MAZE_SIZE);

    Serial.print("East wall at (15,15): ");
    Serial.println(wall_east[15][15]);

    Serial.print("South wall at (15,15): ");
    Serial.println(wall_south[15][15]);
}

void flood_fill() {
    init_manhattan_distances();

    bool changed = true;

    while (changed) {
        changed = false;

        for (int x = 0; x < MAZE_SIZE; x++) {
            for (int y = 0; y < MAZE_SIZE; y++) {
                if (is_goal(x, y)) {
                    flood[x][y] = 0;
                    continue;
                }

                int min_neighbor = 255;

                for (int d = 0; d < 4; d++) {
                    Direction dir = (Direction)d;

                    if (has_wall(x, y, dir)) continue;

                    int nx = neighbor_x(x, dir);
                    int ny = neighbor_y(y, dir);

                    if (!in_bounds(nx, ny)) continue;

                    if (flood[nx][ny] < min_neighbor) {
                        min_neighbor = flood[nx][ny];
                    }
                }

                if (min_neighbor == 255) continue;

                int new_value = min_neighbor + 1;

                if (new_value > 255) {
                    new_value = 255;
                    Serial.println("goal is walled off");
                }

                if (new_value != flood[x][y]) {
                    flood[x][y] = new_value;
                    changed = true;
                }
            }
        }
    }
}

Direction flood_get_best_direction(int x, int y, Direction heading) {
    Direction best_dir = heading;
    int best_value = 30000; // arbitrarily high number
    bool found_valid = false;

    Direction check_order[4] = {
        robot_heading,
        left_of(robot_heading),
        right_of(robot_heading),
        opposite(robot_heading)
    };

    Direction best_dir = robot_heading;
    int best_value = 999;
    bool found_valid_move = false;

    for (int i = 0; i < 4; i++) {
        Direction dir = check_order[i];

        if (has_wall(x, y, dir)) continue;

        int nx = neighbor_x(x, dir);
        int ny = neighbor_y(y, dir);

        if (!in_bounds(nx, ny)) continue;

        if (!found_valid || flood[nx][ny] < best_value) {
            best_value = flood[nx][ny];
            best_dir = dir;
            found_valid = true;
        }
    }

    if (!found_valid_move) {
        return opposite(robot_heading);
    }

    return best_dir;
}

FloodOutput flood_fill_step(
    int x,
    int y,
    int a_global,
    const Micromouse::WallsCurrentCell& walls
) {
    Direction robot_heading = (Direction)a_global;

    Serial.print("Flood step cell: ");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(" heading: ");
    Serial.println((int)robot_heading);

    update_walls_from_tof(x, y, robot_heading, walls);

    Serial.print("N wall: ");
    Serial.println(has_wall(x, y, NORTH));
    Serial.print("E wall: ");
    Serial.println(has_wall(x, y, EAST));
    Serial.print("S wall: ");
    Serial.println(has_wall(x, y, SOUTH));
    Serial.print("W wall: ");
    Serial.println(has_wall(x, y, WEST));

    flood_fill();

    Direction best_dir = flood_get_best_direction(x, y, robot_heading);

    int next_x = neighbor_x(x, best_dir);
    int next_y = neighbor_y(y, best_dir);

    if (!in_bounds(next_x, next_y) || has_wall(x, y, best_dir)) {
        Serial.println("ERROR: flood wanted invalid move. Staying in place.");
        best_dir = robot_heading;
        next_x = x;
        next_y = y;
    }

    FloodOutput output;
    output.want_dir = best_dir;
    output.x_want = next_x;
    output.y_want = next_y;
    output.a_want = (int)best_dir;

    return output;
}
