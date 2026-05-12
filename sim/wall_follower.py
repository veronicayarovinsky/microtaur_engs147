import API
import sys

def log(string):
    sys.stderr.write(string + "\n")

def main():
    log("Running Wall Follower...")

    x, y = 0, 0
    direction = 0  # 0:N, 1:E, 2:S, 3:W

    while True:

        if (x == 7 or x == 8) and (y == 7 or y == 8):
            log("Found the center!")
            break

        if not API.wallRight():
            API.turnRight()
            direction = (direction + 1) % 4
            
        while API.wallFront():
            API.turnLeft()
            direction = (direction - 1) % 4
            
        API.moveForward()

        if direction == 0: y += 1
        elif direction == 1: x += 1
        elif direction == 2: y -= 1
        elif direction == 3: x -= 1

if __name__ == "__main__":
    main()