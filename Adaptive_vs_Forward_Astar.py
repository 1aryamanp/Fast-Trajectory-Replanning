import Maze_Generator as mazeGen
import Repeated_Forward_Astar as fAstar
import Adaptive_Astar as aAstar
import numpy as np

mazeDim = 10
p = .3

cells_f = [] 
times_f = []
cells_a = [] 
times_a = []

for i in range(0, 50):
    OG_maze = mazeGen.create_maze(mazeDim, p, False)
    expanded_cells_f, time_elapsed_f = fAstar.astar_forward(OG_maze, mazeDim, fAstar.tie_breaker_greater, False)
    cells_f.append(expanded_cells_f)
    times_f.append(time_elapsed_f)
    expanded_cells_a, time_elapsed_a = aAstar.astar_adaptive(OG_maze, mazeDim, False)
    cells_a.append(expanded_cells_a)
    times_a.append(time_elapsed_a)

print("Forward A Star")
print("Average expanded cells ", np.mean(cells_f))
print("Standard Deviation of expanded cells", np.std(cells_f))
print("Average runtime ", np.mean(times_f))
print("Total runtime ", np.sum(times_f))

print("\n--------------------------------------------\n")

print("Adaptive A Star")
print("Average expanded cells ", np.mean(cells_a))
print("Standard Deviation of expanded cells", np.std(cells_a))
print("Average runtime ", np.mean(times_a))
print("Total runtime ", np.sum(times_a))

# print("Forwards A star")
# fAstar.astar_forward(maze, dim, fAstar.tie_breaker_smaller, True)

# print("\n--------------------------------------------\n")

# print("Adaptive A star")
# aAstar.astar_adaptive(maze, dim, True)
