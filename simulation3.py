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
YELLOW = (255, 255, 0)   # Item Picked

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
start = (0, 0)         # Top-left corner
item_location = (9, 9)   # Bottom-right corner (Shelf Location)
packing_station = (0, 9) # Now at (0, 8)

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


# First Path: Start to Item
path_to_item = astar(warehouse, start, item_location)

# Second Path: Item to Packing Station
path_to_packing = astar(warehouse, item_location, packing_station)

# Print the paths in the console
if path_to_item and path_to_packing:
    print("\nPath to Item found! The robot will move through these coordinates:")
    print(" -> ".join(str(step) for step in [start] + path_to_item + [item_location]))

    print("\nPath to Packing Station found! The robot will move through these coordinates:")
    print(" -> ".join(str(step) for step in [item_location] + path_to_packing + [packing_station]))
else:
    print("\nNo path found!")
    path_to_item = []  # Handle the case of no path
    path_to_packing = []


# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((COLS * CELL_SIZE, ROWS * CELL_SIZE))
pygame.display.set_caption("Warehouse Robot Pathfinding")

# Main loop
running = True
robot_path = []  # Track the robot's movement
returning = False
item_picked = False
arrived_at_packing = False
picking_delay = False # to add the picking delay

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

    # Draw packing station
    pygame.draw.rect(screen, (0,128,0), (packing_station[1] * CELL_SIZE, packing_station[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    pygame.draw.rect(screen, RED, (item_location[1] * CELL_SIZE, item_location[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    # Move robot along the path
    if path_to_item and not item_picked:
        if robot_path != path_to_item:
            robot_path.append(path_to_item[len(robot_path)])
            time.sleep(0.2)  # Delay for animation
        else:
            item_picked = True
            picking_delay = True #Start picking delay
            robot_path = []  # Clear path for return journey

        for step in robot_path:
            pygame.draw.rect(screen, BLUE, (step[1] * CELL_SIZE, step[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    elif item_picked and path_to_packing and not arrived_at_packing and not picking_delay:
        pygame.draw.rect(screen, YELLOW, (item_location[1] * CELL_SIZE, item_location[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

        if robot_path != path_to_packing:
            robot_path.append(path_to_packing[len(robot_path)])
            time.sleep(0.2)

        else:
            arrived_at_packing = True

        for step in robot_path:
            pygame.draw.rect(screen, BLUE, (step[1] * CELL_SIZE, step[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    elif picking_delay:
        pygame.draw.rect(screen, YELLOW, (item_location[1] * CELL_SIZE, item_location[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))
        time.sleep(1)  # Simulate the robot picking up the item for 1 second
        picking_delay = False #Stop picking delay

    else:
        running = False  # Terminate when arrived at packing

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.flip()

pygame.quit()