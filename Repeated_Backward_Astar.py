from HeapPriorityQueue import HeapPriorityQueue
import time
import random as RandomValue
import matplotlib.pyplot as plt
import seaborn as sns
import copy
import numpy as np
import Maze_Generator as mazeGen
from matplotlib.colors import ListedColormap

def get_path(var, start, goal):  # returns the intermediate paths for each Astar call
    list = []
    key = goal
    while True:
        list.append(key)
        value = var.get(key)
        key = value
        if key == start:
            list.append(key)
            break

    for k in range(0, len(list)):
        x, y = list[k]

    return list


def final_path_visualizer(maze, var, start, goal):  # prints the final path from source to destination
    maze4 = copy.deepcopy(maze)
    ln = var
    for k in range(0, len(ln)):
        x, y = ln[k]
        maze4[x][y] = 2
        
    maze4[goal] = 4
    maze4[start] = 3

    cmap = ListedColormap(['black', 'white', 'yellow', 'green', 'red'])
    
    plt.figure(figsize=(5, 5))
    plt.title('Maze Visualization')
    sns.heatmap(maze4, linewidths=0.1, linecolor="gray", square=True, cbar=False, xticklabels=False,
                yticklabels=False, cmap=cmap, vmin=0, vmax=4)
    plt.show()


def all_actions(var, maze, dim):  # returns the neighbours for current cell and blocked cells
    actions = ()
    blocks = []
    i, j = var
    neighbor = [(i, j + 1), (i, j - 1), (i + 1, j), (i - 1, j)]

    if (j + 1 < dim) and ((maze[neighbor[0]] == 0) and (not (neighbor[0] in blocks))):
        blocks.append(neighbor[0])
    if j - 1 >= 0 and ((maze[neighbor[1]] == 0) and (not (neighbor[1]in blocks))):
        blocks.append(neighbor[1])
    if (i + 1 < dim) and ((maze[neighbor[2]] == 0) and (not (neighbor[2] in blocks))):
        blocks.append(neighbor[2])
    if i - 1 >= 0 and ((maze[neighbor[3]] == 0) and (not (neighbor[3] in blocks))):
        blocks.append(neighbor[3])

    if j + 1 < dim and (maze[neighbor[0]] != 0):
        actions += (neighbor[0],)
    if j - 1 >= 0 and (maze[neighbor[1]] != 0):
        actions += (neighbor[1],)
    if i + 1 < dim and (maze[neighbor[2]] != 0):
        actions += (neighbor[2],)
    if i - 1 >= 0 and (maze[neighbor[3]] != 0):
        actions += (neighbor[3],)

    return actions, blocks

def tie_breaker_greater(open_list, g):
    val = open_list.min()
    (v1, v2) = open_list.min_value()
    lq = open_list.tolist(val)
    for i in range(0, len(lq)):
        y, (z1, z2) = lq[i]
        if g[z1][z2] >= g[v1][v2]:
            n, z = open_list.search_Queue(y, (z1, z2))
    return open_list.remove_g_index(n)

def compute_path_backward(search, open_list, closed_list, g, f, h, maze, counter, dim):
    search_path = {}
    ct = 0
    while (not open_list.is_empty()) and (g[0][0] > open_list.min()):
        ct = ct + 1
        var = tie_breaker_greater(open_list, g)
        a, b = var
        closed_list.append(var)
        actions = ()
        actions, blocks = all_actions(var, maze, dim)

        for i in range(0, len(actions)):
            x, y = actions[i]
            if search[x][y] < counter:
                g[x][y] = 1000
                search[x][y] = counter

            if g[x][y] > g[a][b] + 1:
                g[x][y] = g[a][b] + 1
                search_path[(x, y)] = (a, b)
                f[x][y] = g[x][y] + h[x][y]
                p, q = open_list.search_Queue(f[x][y], (x, y))
                if not open_list.is_empty() and (x, y) == q:
                    open_list.remove_index(p)
                open_list.add(f[x][y], (x, y))
    return search_path


def astar_backward(mazeOriginal, dim, do_visual):
    start_time = time.time()  # Start timing the execution
    maze = copy.deepcopy(mazeOriginal)  # Create a deep copy to preserve the original maze
    mazer = np.ones((dim, dim))  # Initialize a maze for visualization
    counter = 0  # Initialize a step counter
    search = np.zeros((dim, dim))  # Track visited cells
    my_visitors = []  # Track the cells visited during the search
    path_exists = True  # Flag to indicate if a path exists
    h = np.zeros((dim, dim))  # Heuristic function array
    g = np.zeros((dim, dim))  # Cost to reach current cell
    f = np.zeros((dim, dim))  # Estimated total cost (f = g + h)
    size = dim - 1  # Adjust for zero-based indexing

    # Initialize heuristic values based on Manhattan distance
    for i in range(dim):
        for j in range(dim):
            h[i][j] = abs(0 - i) + abs(0 - j)

    open_list = HeapPriorityQueue()  # Initialize priority queue
    start = (size, size)  # Set starting point
    agent = start  # Current position of the agent
    goal = (0, 0)  # Set goal position
    
    # Begin search loop
    while agent != goal:
        g = np.zeros((dim, dim))  # Reset cost array
        f = np.zeros((dim, dim))  # Reset total cost array
        c, d = agent  # Current agent position
        g[c][d] = 0  # Set start cell cost to 0
        counter += 1  # Increment step counter
        search[c][d] = counter  # Mark current cell as visited
        g[goal] = 1000  # Set high initial goal cost
        search[goal] = counter  # Mark goal as visited for this search
        open_list.remove_all()  # Clear priority queue
        closed_list = []  # Initialize closed list
        
        f[c][d] = g[c][d] + h[c][d]  # Calculate f value for start cell
        open_list.add(f[c][d], (c, d))  # Add start cell to open list
        var = compute_path_backward(search, open_list, closed_list, g, f, h, mazer, counter, dim)
        
        # Check if a path was found
        if open_list.is_empty():
            if do_visual:
                print("Pathfinding failed: No path exists from start to goal.")
                path_exists = False
            break
        
        nodes = get_path(var, agent, goal)  # Retrieve path

        # Update maze based on found path
        for i in reversed(range(len(nodes))):
            x, y = nodes[i]
            
            found = False
            actions, blocks = all_actions((x, y), maze, dim)
            for k in blocks:
                a, b = k
                mazer[a][b] = 0  # Block cell in visualization maze
                if (a, b) in nodes:
                    found = True  # Break if a block interrupts the path
            if found:
                break
            my_visitors.append((x, y))  # Add cell to visited list
        
        agent = (x, y)  # Update agent position

    end_time = time.time()  # End timing
    total_time = end_time - start_time  # Calculate total execution time

    if do_visual:
        # Provide a summary of the search process
        print(f"Counter for backward A*: {counter}")
        print(f"Number of cells expanded: {len(my_visitors)}")
        final_path_visualizer(mazer, my_visitors, start, goal)  # Visualize final path
        print(f"Total computation time: {total_time:.2f} seconds")

    open_list.remove_all()  # Clear priority queue

    if path_exists:
        return len(my_visitors), total_time  # Return path length and total time if path exists

if __name__ == "__main__":
    mazeDim = 10
    p = 0.3
    OG_maze = mazeGen.create_maze(mazeDim, p, True)  # Generate maze
    result = astar_backward(OG_maze, mazeDim, True)  # Run A* backward search
    
    # Print final result
    if result:
        expanded_cells, total_time = result
        print(f"Path found! Cells expanded: {expanded_cells}, Time taken: {total_time:.2f} seconds.")
    else:
        print("Failed to find a path.")

