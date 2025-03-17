import pygame
import heapq
import time

# Define grid size
ROWS, COLS = 10, 10  # Warehouse grid size
CELL_SIZE = 50       # Cell size in pixels

# Define colors
WHITE = (255, 255, 255)  # Open path
BLACK = (0, 0, 0)        # Blocked area
GREEN = (0, 255, 0)      # Start position
RED = (255, 0, 0)        # Goal position
BLUE = (0, 0, 255)       # Path taken by robot

# Define the warehouse grid (0 = open path, 1 = blocked)
warehouse = [
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [1, 1, 0, 1, 1, 0, 1, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 0, 1],
    [0, 1, 1, 1, 1, 1, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 1, 1, 0],
    [1, 1, 1, 1, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 1, 1, 1, 1, 0, 0, 0],
    [0, 1, 0, 0, 0, 0, 1, 1, 1, 0],
    [0, 1, 1, 1, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 1, 1, 0]
]

# Start and goal positions
start = (0, 0)  # Top-left corner
goal = (9, 9)   # Bottom-right corner

# Define possible moves (Up, Down, Left, Right)
MOVES = [(0, 1), (0, -1), (1, 0), (-1, 0)]


# Heuristic function: Manhattan distance
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# A* Search Algorithm
def astar(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))  # Priority queue with (cost, node)
    
    came_from = {}  # Path reconstruction
    g_score = {node: float('inf') for row in grid for node in enumerate(row)}
    g_score[start] = 0
    
    f_score = {node: float('inf') for row in grid for node in enumerate(row)}
    f_score[start] = heuristic(start, goal)

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        for dx, dy in MOVES:
            neighbor = (current[0] + dx, current[1] + dy)

            if 0 <= neighbor[0] < ROWS and 0 <= neighbor[1] < COLS:
                if grid[neighbor[0]][neighbor[1]] == 1:  # Skip blocked cells
                    continue

                tentative_g_score = g_score[current] + 1

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found


# Find the shortest path
path = astar(warehouse, start, goal)

# Print the path in the console
if path:
    print("\nPath found! The robot will move through these coordinates:")
    print(" -> ".join(str(step) for step in [start] + path + [goal]))
else:
    print("\nNo path found!")

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((COLS * CELL_SIZE, ROWS * CELL_SIZE))
pygame.display.set_caption("Warehouse Robot Pathfinding")

# Main loop
running = True
robot_path = []  # Track the robot's movement

while running:
    screen.fill(WHITE)

    for row in range(ROWS):
        for col in range(COLS):
            cell = warehouse[row][col]
            color = WHITE if cell == 0 else BLACK

            pygame.draw.rect(screen, color, (col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE))
            pygame.draw.rect(screen, (200, 200, 200), (col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE), 1)

    # Draw start and goal
    pygame.draw.rect(screen, GREEN, (start[1] * CELL_SIZE, start[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))
    pygame.draw.rect(screen, RED, (goal[1] * CELL_SIZE, goal[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    # Move robot along the path
    if path:
        if robot_path != path:
            robot_path.append(path[len(robot_path)])
            time.sleep(0.2)  # Delay for animation

        for step in robot_path:
            pygame.draw.rect(screen, BLUE, (step[1] * CELL_SIZE, step[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.flip()

pygame.quit()
