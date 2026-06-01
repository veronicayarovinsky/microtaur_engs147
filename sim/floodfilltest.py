import sys
import API

# --- Constants & Enums ---
NORTH = 0
EAST = 1
SOUTH = 2
WEST = 3

# Mackorone mms directions use characters
DIR_TO_CHAR = {NORTH: 'n', EAST: 'e', SOUTH: 's', WEST: 'w'}

MAZE_SIZE = 16

def dir_name(direction):
    if direction == NORTH: return "NORTH"
    if direction == EAST:  return "EAST"
    if direction == SOUTH: return "SOUTH"
    if direction == WEST:  return "WEST"
    return "?"

# --- Data Structures ---
class Pose:
    def __init__(self, x=0, y=0, a=NORTH):
        self.x = x
        self.y = y
        self.a = a

class FloodOutput:
    def __init__(self, want_dir=NORTH, x_want=0, y_want=0, a_want=NORTH):
        self.want_dir = want_dir
        self.x_want = x_want
        self.y_want = y_want
        self.a_want = a_want

# --- Flood Fill Logic ---
class FloodFill:
    def __init__(self):
        self.flood = [[255 for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)]
        self.walls = {
            NORTH: [[False for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)],
            EAST:  [[False for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)],
            SOUTH: [[False for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)],
            WEST:  [[False for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)]
        }
        self.init_maze()

    def in_bounds(self, x, y):
        return 0 <= x < MAZE_SIZE and 0 <= y < MAZE_SIZE

    def has_wall(self, x, y, direction):
        if not self.in_bounds(x, y):
            return True
        return self.walls[direction][x][y]

    def neighbor_x(self, x, direction):
        if direction == EAST: return x + 1
        if direction == WEST: return x - 1
        return x

    def neighbor_y(self, y, direction):
        if direction == NORTH: return y + 1
        if direction == SOUTH: return y - 1
        return y

    def left_of(self, direction):
        return (direction + 3) % 4

    def right_of(self, direction):
        return (direction + 1) % 4

    def maze_set_wall(self, x, y, direction, exists):
        if not self.in_bounds(x, y):
            return

        # Update internal logic
        if direction == NORTH:
            self.walls[NORTH][x][y] = exists
            if self.in_bounds(x, y + 1): self.walls[SOUTH][x][y + 1] = exists
        elif direction == EAST:
            self.walls[EAST][x][y] = exists
            if self.in_bounds(x + 1, y): self.walls[WEST][x + 1][y] = exists
        elif direction == SOUTH:
            self.walls[SOUTH][x][y] = exists
            if self.in_bounds(x, y - 1): self.walls[NORTH][x][y - 1] = exists
        elif direction == WEST:
            self.walls[WEST][x][y] = exists
            if self.in_bounds(x - 1, y): self.walls[EAST][x - 1][y] = exists
            
        # Draw the discovered wall in the simulator UI
        if exists:
            API.setWall(x, y, DIR_TO_CHAR[direction])

    def update_walls_from_tof(self, x, y, heading, front, left, right):
        if front != -1:
            self.maze_set_wall(x, y, heading, front == 1)
        if left != -1:
            self.maze_set_wall(x, y, self.left_of(heading), left == 1)
        if right != -1:
            self.maze_set_wall(x, y, self.right_of(heading), right == 1)

    def init_maze(self):
        for x in range(MAZE_SIZE):
            for y in range(MAZE_SIZE):
                self.flood[x][y] = 255
                self.walls[NORTH][x][y] = False
                self.walls[EAST][x][y] = False
                self.walls[SOUTH][x][y] = False
                self.walls[WEST][x][y] = False

        # Set outer boundaries
        for i in range(MAZE_SIZE):
            self.walls[SOUTH][i][0] = True
            self.walls[NORTH][i][MAZE_SIZE - 1] = True
            self.walls[WEST][0][i] = True
            self.walls[EAST][MAZE_SIZE - 1][i] = True

    def fill(self):
        for x in range(MAZE_SIZE):
            for y in range(MAZE_SIZE):
                self.flood[x][y] = 255

        goals_x = [7, 7, 8, 8]
        goals_y = [7, 8, 7, 8]
        queue = []

        for i in range(4):
            gx = goals_x[i]
            gy = goals_y[i]
            self.flood[gx][gy] = 0
            queue.append((gx, gy))

        while len(queue) > 0:
            x, y = queue.pop(0)
            current_value = self.flood[x][y]

            for d in range(4):
                if self.has_wall(x, y, d):
                    continue

                nx = self.neighbor_x(x, d)
                ny = self.neighbor_y(y, d)

                if not self.in_bounds(nx, ny):
                    continue

                if self.flood[nx][ny] > current_value + 1:
                    self.flood[nx][ny] = current_value + 1
                    queue.append((nx, ny))
                    
        # Visually update flood fill values in simulator
        for x in range(MAZE_SIZE):
            for y in range(MAZE_SIZE):
                API.setText(x, y, str(self.flood[x][y]))

    def get_best_direction(self, x, y):
        best_dir = NORTH
        best_value = 255

        for d in range(4):
            if self.has_wall(x, y, d):
                continue

            nx = self.neighbor_x(x, d)
            ny = self.neighbor_y(y, d)

            if not self.in_bounds(nx, ny):
                continue

            if self.flood[nx][ny] < best_value:
                best_value = self.flood[nx][ny]
                best_dir = d

        return best_dir

    def step(self, x, y, heading, front, left, right):
        self.update_walls_from_tof(x, y, heading, front, left, right)
        self.fill()
        best_dir = self.get_best_direction(x, y)

        return FloodOutput(
            want_dir=best_dir,
            x_want=self.neighbor_x(x, best_dir),
            y_want=self.neighbor_y(y, best_dir),
            a_want=best_dir
        )

# --- Navigation Helpers ---
def orient_robot(current_heading, target_heading):
    """Issues simulator turning commands to reach target heading."""
    diff = (target_heading - current_heading) % 4
    
    if diff == 1:
        API.turnRight()
    elif diff == 2:
        API.turnRight()
        API.turnRight()
    elif diff == 3:
        API.turnLeft()

# --- Main Simulator Loop ---
def main():
    API.log("Starting Maze Solver...")
    
    # Optional API check: if simulator provides size, you can assert it's 16
    # MAZE_SIZE = API.mazeWidth()
    
    flood_algorithm = FloodFill()
    pose = Pose(x=0, y=0, a=NORTH)
    
    API.setColor(0, 0, 'G') # Mark start green
    
    while True:
        # 1. Check goal condition
        if pose.x in [7, 8] and pose.y in [7, 8]:
            API.log("Goal Reached!")
            API.setColor(pose.x, pose.y, 'Y')
            break
            
        # 2. Read walls from simulator API
        front_wall = 1 if API.wallFront() else 0
        left_wall  = 1 if API.wallLeft() else 0
        right_wall = 1 if API.wallRight() else 0
        
        # 3. Calculate next step
        next_move = flood_algorithm.step(
            pose.x, 
            pose.y, 
            pose.a, 
            front_wall, 
            left_wall, 
            right_wall
        )
        
        # 4. Turn virtual robot to match calculated desired direction
        orient_robot(pose.a, next_move.want_dir)
        
        # 5. Move virtual robot forward
        API.moveForward()
        
        # 6. Update local state
        pose.x = next_move.x_want
        pose.y = next_move.y_want
        pose.a = next_move.want_dir # Heading is now the direction we moved

if __name__ == "__main__":
    main()