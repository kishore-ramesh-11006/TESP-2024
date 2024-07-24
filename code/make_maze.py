import random
import numpy as np
from PIL import Image, ImageDraw

# this program is so-called making maze.
# The task is to create a maze, and the goal is to output the maze image when this program is executed.

# Set the height and width of the maze
height, width = 15, 15    # if changing these numbers, the overall size of the maze is changed.
maze_base = [[1]*width for _ in range(height)]

# Set the start point to a path(0).
maze_base[1][1] = 0

# Vectors for the x and y axes (directions to carve the path)
dx = [(1,2), (-1,-2), (0,0), (0,0)] #set x vectors
dy = [(0,0), (0,0), (1,2), (-1,-2)] #set y vectors

# List of directions to move
DIRECTIONS = [(0, 1), (1, 0), (0, -1), (-1, 0)]

def make_maze(ny, nx):
    """
    Recursive function to create the maze
    """
    array = list(range(4)) 
    random.shuffle(array) # Randomly decide the direction to move

    for i in array:
        # Skip if the next point exceeds the boundary
        if ny+dy[i][1]<1 or ny+dy[i][1]>=height: 
            continue

        if nx+dx[i][1]<1 or nx+dx[i][1]>=width: 
            continue

        if maze_base[ny+dy[i][1]][nx+dx[i][1]]==0:
            continue
        
        # Create the code to carve the path
        for j in range(2):
            """
            ~~~~~~~~~~~~
            ~~~~~~~~~~~~
            """

        # Move to the carved point and call the function recursively
        make_maze(ny+dy[i][1], nx+dx[i][1])

def bfs(start):
    """
    Function to calculate the distance to each point in the maze using breadth-first search
    """
    # Initialize the queue and distances with the start point having distance 0
    queue = [start]
    distances = {start: 0}

    # While there are points to explore
    while queue:
        x, y = queue.pop(0)

        # Check all four directions (right, down, left, up)
        for dx, dy in DIRECTIONS: 
            nx, ny = x + dx, y + dy

            # Check new coordinates and position are within bounds
            if (0 <= nx < width and 0 <= ny < height and
                maze_base[ny][nx] == 0 and (nx, ny) not in distances): 
                queue.append((nx, ny))
                distances[(nx, ny)] = distances[(x, y)] + 1
    return distances

def save_maze_as_png(binary_maze, start, goal, filename):
    """
    Function to save the maze as a PNG image
    """
    height = len(binary_maze)
    width = len(binary_maze[0])
    image = Image.new('L', (width, height))
    for y in range(height):
        for x in range(width):
            image.putpixel((x, y), 255 * binary_maze[y][x])
    
    # Create marking the start and goal points
    draw = ImageDraw.Draw(image)
    """
    ~~~~~~~~~~~~~~~
    ~~~~~~~~~~~~~~~
    """

    image.save(filename)

# Set the start point
start_x = 1
start_y = 1

# Create the maze
make_maze(start_y, start_x)

# Calculate the distance to each point in the maze
distances = bfs((start_x, start_y))
# Set the farthest point as the goal
farthest_point = max(distances, key=distances.get)

# Enlarge the maze size (100 times)
maze_base = np.array(maze_base)
maze_base = np.repeat(maze_base, 100, axis=0)
maze_base = np.repeat(maze_base, 100, axis=1)
maze_base = maze_base.tolist()

# Prompt for the date input
print("enter yyyymmdd")
date = input()

# Save the maze image with the start and goal points marked
save_maze_as_png(maze_base, (start_x, start_y), farthest_point, 'C:/Users/gkeik/Documents/TESP2024_v1/TESP2024_v1/make_maze/maze_' + str(date) + '.png')

# Display the start point and the goal point
print("start=({}, {})".format(start_x, start_y))
print("goal=({}, {})".format(farthest_point[0], farthest_point[1]))