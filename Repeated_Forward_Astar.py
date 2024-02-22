from HeapPriorityQueue import HeapPriorityQueue
import time
import random
import matplotlib.pyplot as plt
import seaborn as sns
import copy
import numpy as np
import Maze_Generator as mazeGen

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

    plt.figure(3, figsize=(5, 5))
    ax = sns.heatmap(maze4, linewidths=0.1, linecolor="gray", square=True, cbar=False, xticklabels=False,
                     yticklabels=False, cmap=sns.diverging_palette(10, 220, n=200), vmin=0, vmax=4)
    plt.show()


def all_actions(var, maze, dim):  # returns the neighbours for current cell and blocked cells
    actions = ()
    blocks = []
    i, j = var
    neighbor = [(i, j + 1), (i, j - 1), (i + 1, j), (i - 1, j)]

    if (j + 1 < dim) and ((maze[neighbor[0]] == 0) and (not (neighbor[0] in blocks))):
        blocks.append(neighbor[0])
    if j - 1 >= 0 and ((maze[neighbor[1]] == 0) and (not (neighbor[1] in blocks))):
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

def tie_breaker_smaller(open_list, g):
    val = open_list.min()
    (v1, v2) = open_list.min_value()
    lq = open_list.tolist(val)
    for i in range(0, len(lq)):
        y, (z1, z2) = lq[i]
        if g[z1][z2] <= g[v1][v2]:
            n, z = open_list.search_Queue(y, (z1, z2))
    return open_list.remove_g_index(n)

def compute_path_forward(search, open_list, closed_list, g, f, h, maze, counter, size, tie_breaker):
    search_path = {}
    ct = 0
    while (not open_list.is_empty()) and (g[size][size] > open_list.min()):
        ct = ct + 1
        var = tie_breaker(open_list, g)
        a, b = var
        closed_list.append(var)
        actions = ()
        actions, blocks = all_actions(var, maze, size + 1)

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


def astar_forward(mazeOriginal, dim, tie_breaker_function, do_visual):
    start_time = time.time()
    maze = copy.deepcopy(mazeOriginal)
    mazer = np.ones((dim, dim))
    counter = 0
    search = np.zeros((dim, dim))
    my_visitors = []
    path_exists = True
    h = np.zeros((dim, dim))
    g = np.zeros((dim, dim))
    f = np.zeros((dim, dim))
    size = dim - 1

    for i in range(0, dim):
        for j in range(0, dim):
            h[i][j] = (abs(size - i) + abs(size - j))

    open_list = HeapPriorityQueue()
    start = (0, 0)
    agent = start
    goal = (size, size)
    p, q = goal

    while agent != goal:
        g = np.zeros((dim, dim))
        f = np.zeros((dim, dim))
        c, d = agent
        g[c][d] = 0
        counter = counter + 1
        search[c][d] = counter
        g[p][q] = 1000
        search[p][q] = counter
        open_list.remove_all()
        closed_list = []
        f[c][d] = g[c][d] + h[c][d]
        open_list.add(f[c][d], (c, d))
        var = compute_path_forward(search, open_list, closed_list, g, f, h, mazer, counter, size, tie_breaker_function)
        if open_list.is_empty():
            if do_visual:
                print("No path found")
                path_exists = False
            break
        nodes = get_path(var, agent, goal)

        for i in range(len(nodes) - 1, -1, -1):
            x, y = nodes[i]

            found = False
            actions, blocks = all_actions((x, y), maze, dim)
            for k in blocks:
                a, b = k
                mazer[a][b] = 0
                if (a, b) in nodes:
                    found = True
            if found:
                break
            my_visitors.append((x, y))

        agent = (x, y)

    end_time = time.time()
    total_time = end_time - start_time

    if do_visual:
        print("Counter for forward Astar ", counter)
        print("Expanded cells for forward Astar ", len(my_visitors))
        final_path_visualizer(mazer, my_visitors, start, goal)
        print("Total elapsed  ", total_time, " seconds")
    open_list.remove_all()

    if (path_exists):
        return len(my_visitors), total_time


if __name__ == "__main__":
    mazeDim = 10
    p = .3
    OG_maze = mazeGen.create_maze(mazeDim, p, False)
    print(astar_forward(OG_maze, mazeDim, tie_breaker_greater, True))

    # cells = [] 
    # times = []
    # for i in range(0, 5):
    #     OG_maze = mazeGen.create_maze(mazeDim, p, False)
    #     expanded_cells, time_elapsed = astar_forward(OG_maze, mazeDim, False, tie_breaker_smaller)
    #     cells.append(expanded_cells)
    #     times.append(time_elapsed)
    # print("Average expanded cells ", np.mean(cells))
    # print("Standard Deviation of expanded cells", np.std(cells))
    # print("Average runtime ", np.mean(times))
    # print("Total runtime ", np.sum(times))
