// Flood Fill Simulation for Arduino Due
// Prints a 16x16 flood fill distance grid and simulated robot path

#define SIZE 16
#define INF 255

enum Direction {
  NORTH = 0,
  EAST = 1,
  SOUTH = 2,
  WEST = 3
};

struct Cell {
  bool wall[4];
  int distance;
};

Cell maze[SIZE][SIZE];

void initializeMaze() {
  for (int y = 0; y < SIZE; y++) {
    for (int x = 0; x < SIZE; x++) {
      maze[y][x].distance = INF;

      for (int d = 0; d < 4; d++) {
        maze[y][x].wall[d] = false;
      }
    }
  }

  // Add outer boundary walls
  for (int i = 0; i < SIZE; i++) {
    maze[0][i].wall[SOUTH] = true;
    maze[SIZE - 1][i].wall[NORTH] = true;
    maze[i][0].wall[WEST] = true;
    maze[i][SIZE - 1].wall[EAST] = true;
  }
}

void setGoal() {
  // Center goal cells for 16x16 maze
  maze[7][7].distance = 0;
  maze[7][8].distance = 0;
  maze[8][7].distance = 0;
  maze[8][8].distance = 0;
}

void floodFill() {
  bool changed = true;

  while (changed) {
    changed = false;

    for (int y = 0; y < SIZE; y++) {
      for (int x = 0; x < SIZE; x++) {

        // Do not update goal cells
        if (maze[y][x].distance == 0) {
          continue;
        }

        int minNeighbor = INF;

        if (!maze[y][x].wall[NORTH] && y < SIZE - 1) {
          minNeighbor = min(minNeighbor, maze[y + 1][x].distance);
        }

        if (!maze[y][x].wall[EAST] && x < SIZE - 1) {
          minNeighbor = min(minNeighbor, maze[y][x + 1].distance);
        }

        if (!maze[y][x].wall[SOUTH] && y > 0) {
          minNeighbor = min(minNeighbor, maze[y - 1][x].distance);
        }

        if (!maze[y][x].wall[WEST] && x > 0) {
          minNeighbor = min(minNeighbor, maze[y][x - 1].distance);
        }

        int newDistance = minNeighbor + 1;

        if (maze[y][x].distance != newDistance) {
          maze[y][x].distance = newDistance;
          changed = true;
        }
      }
    }
  }
}

void printMaze() {
  Serial.println();
  Serial.println("Flood Fill Distance Grid:");

  for (int y = SIZE - 1; y >= 0; y--) {
    for (int x = 0; x < SIZE; x++) {
      Serial.print(maze[y][x].distance);
      Serial.print("\t");
    }
    Serial.println();
  }
}

Direction chooseNextMove(int x, int y) {
  int bestDistance = maze[y][x].distance;
  Direction bestDir = NORTH;

  if (!maze[y][x].wall[NORTH] &&
      y < SIZE - 1 &&
      maze[y + 1][x].distance < bestDistance) {
    bestDistance = maze[y + 1][x].distance;
    bestDir = NORTH;
  }

  if (!maze[y][x].wall[EAST] &&
      x < SIZE - 1 &&
      maze[y][x + 1].distance < bestDistance) {
    bestDistance = maze[y][x + 1].distance;
    bestDir = EAST;
  }

  if (!maze[y][x].wall[SOUTH] &&
      y > 0 &&
      maze[y - 1][x].distance < bestDistance) {
    bestDistance = maze[y - 1][x].distance;
    bestDir = SOUTH;
  }

  if (!maze[y][x].wall[WEST] &&
      x > 0 &&
      maze[y][x - 1].distance < bestDistance) {
    bestDistance = maze[y][x - 1].distance;
    bestDir = WEST;
  }

  return bestDir;
}

void printDirection(Direction dir) {
  if (dir == NORTH) Serial.println("NORTH");
  else if (dir == EAST) Serial.println("EAST");
  else if (dir == SOUTH) Serial.println("SOUTH");
  else if (dir == WEST) Serial.println("WEST");
}

void simulatePath() {
  int x = 0;
  int y = 0;

  Serial.println();
  Serial.println("Robot Path:");

  while (!(x >= 7 && x <= 8 && y >= 7 && y <= 8)) {
    Serial.print("(");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.println(")");

    Direction nextMove = chooseNextMove(x, y);

    Serial.print("Next Move: ");
    printDirection(nextMove);

    if (nextMove == NORTH) y++;
    else if (nextMove == EAST) x++;
    else if (nextMove == SOUTH) y--;
    else if (nextMove == WEST) x--;

    delay(200);
  }

  Serial.println();
  Serial.print("(");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.println(")");
  Serial.println("Goal Reached!");
}

void setup() {
  Serial.begin(115200);
  delay(3000);

  // Creates space so old Serial Monitor text is easier to ignore
  for (int i = 0; i < 20; i++) {
    Serial.println();
  }

  Serial.println("===== NEW FLOOD FILL SIMULATION RUN =====");

  initializeMaze();
  setGoal();
  floodFill();
  printMaze();
  simulatePath();
}

void loop() {
  // Nothing here because this is a one-time simulation
}