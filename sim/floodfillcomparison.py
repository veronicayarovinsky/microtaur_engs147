import API
import sys

# --- Constants ---
NORTH = 0
EAST = 1
SOUTH = 2
WEST = 3

# Movement Deltas mapping to [NORTH, EAST, SOUTH, WEST]
# This replaces the need for neighbor_x and neighbor_y functions
DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]

DIR_CHARS = ['n', 'e', 's', 'w']
MAZE_SIZE = 16
GOALS = [(7,7), (7,8), (8,7), (8,8)]

def log(msg):
    """Helper to log text to the simulator UI"""
    sys.stderr.write(f"{msg}\n")
    sys.stderr.flush()

class Micromouse:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.heading = NORTH
        
        # 2D array for flood fill distances
        self.distances = [[255 for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)]
        
        # 3D array for walls: walls[x][y][direction]
        self.walls = [[[False for _ in range(4)] for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)]
        
        self._init_perimeter_walls()

    def _init_perimeter_walls(self):
        """Sets the hard outer boundary walls of the maze."""
        for i in range(MAZE_SIZE):
            self.set_wall(i, 0, SOUTH)
            self.set_wall(i, MAZE_SIZE - 1, NORTH)
            self.set_wall(0, i, WEST)
            self.set_wall(MAZE_SIZE - 1, i, EAST)

    def set_wall(self, x, y, direction):
        """Records a wall in the internal map and updates the neighboring cell's wall."""
        if 0 <= x < MAZE_SIZE and 0 <= y < MAZE_SIZE:
            self.walls[x][y][direction] = True
            API.setWall(x, y, DIR_CHARS[direction]) # Draw in simulator
            
            # Also set the opposite wall for the adjacent cell
            nx = x + DX[direction]
            ny = y + DY[direction]
            if 0 <= nx < MAZE_SIZE and 0 <= ny < MAZE_SIZE:
                self.walls[nx][ny][(direction + 2) % 4] = True

    def scan_walls(self):
        """Reads simulator sensors and maps them to absolute directions based on current heading."""
        if API.wallFront():
            self.set_wall(self.x, self.y, self.heading)
        if API.wallRight():
            self.set_wall(self.x, self.y, (self.heading + 1) % 4)
        if API.wallLeft():
            self.set_wall(self.x, self.y, (self.heading + 3) % 4)

    def flood_fill(self):
        """Runs Breadth-First Search (BFS) to calculate shortest paths from goals to all cells."""
        # Reset all distances
        for i in range(MAZE_SIZE):
            for j in range(MAZE_SIZE):
                self.distances[i][j] = 255
                
        queue = []
        
        # Initialize goals with distance 0 and push to queue
        for gx, gy in GOALS:
            self.distances[gx][gy] = 0
            queue.append((gx, gy))
            
        # BFS expansion
        while queue:
            cx, cy = queue.pop(0)
            current_dist = self.distances[cx][cy]
            
            # Check all 4 absolute directions
            for d in range(4):
                if not self.walls[cx][cy][d]: # If there is NO wall blocking the path
                    nx = cx + DX[d]
                    ny = cy + DY[d]
                    
                    if 0 <= nx < MAZE_SIZE and 0 <= ny < MAZE_SIZE:
                        if self.distances[nx][ny] > current_dist + 1:
                            self.distances[nx][ny] = current_dist + 1
                            queue.append((nx, ny))
                            
        # Draw distances in the simulator UI
        for i in range(MAZE_SIZE):
            for j in range(MAZE_SIZE):
                API.setText(i, j, str(self.distances[i][j]))

    def get_best_move(self):
        """
        Looks at accessible neighbors and picks the one with the lowest distance.
        OPTIMIZATION: Evaluates neighbors in order: Straight, Right, Left, Back.
        This tie-breaks equal distances by preferring fewer turns.
        """
        best_dir = self.heading
        min_dist = 255
        
        # Turn offsets: 0=Straight, 1=Right, 3=Left, 2=Back
        for turn_offset in [0, 1, 3, 2]:
            d = (self.heading + turn_offset) % 4
            
            if not self.walls[self.x][self.y][d]:
                nx = self.x + DX[d]
                ny = self.y + DY[d]
                
                if 0 <= nx < MAZE_SIZE and 0 <= ny < MAZE_SIZE:
                    if self.distances[nx][ny] < min_dist:
                        min_dist = self.distances[nx][ny]
                        best_dir = d
                        
        return best_dir

    def move(self, target_dir):
        """Turns the physical robot to face the target direction, then moves forward."""
        diff = (target_dir - self.heading) % 4
        
        if diff == 1:
            API.turnRight()
        elif diff == 2:
            API.turnRight()
            API.turnRight()
        elif diff == 3:
            API.turnLeft()
            
        self.heading = target_dir
        API.moveForward()
        
        # Update internal coordinates
        self.x += DX[self.heading]
        self.y += DY[self.heading]

    def solve(self):
        """Main loop for the micromouse."""
        log("Starting BFS Flood Fill Algorithm...")
        API.setColor(0, 0, 'G')
        
        while True:
            if (self.x, self.y) in GOALS:
                log(f"Goal Reached at ({self.x}, {self.y})!")
                API.setColor(self.x, self.y, 'Y')
                break
                
            self.scan_walls()
            self.flood_fill()
            
            best_dir = self.get_best_move()
            self.move(best_dir)

def main():
    mouse = Micromouse()
    mouse.solve()

if __name__ == "__main__":
    main()