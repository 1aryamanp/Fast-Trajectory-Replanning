'''
import Maze_Generator as mazeGen
import Repeated_Forward_Astar as fAstar
import numpy as np

mazeDim = 51
p = .3

cells_s = [] 
times_s = []
cells_g = [] 
times_g = []

for i in range(0, 50):
    OG_maze = mazeGen.create_maze(mazeDim, p)
    expanded_cells_s, time_elapsed_s = fAstar.astar_forward(OG_maze, mazeDim, fAstar.tie_breaker_smaller, False)
    cells_s.append(expanded_cells_s)
    times_s.append(time_elapsed_s)
    expanded_cells_g, time_elapsed_g = fAstar.astar_forward(OG_maze, mazeDim, fAstar.tie_breaker_greater, False)
    cells_g.append(expanded_cells_g)
    times_g.append(time_elapsed_g)

print("Forward A Star, smaller tie breaker")
print("Average expanded cells ", np.mean(cells_s))
print("Standard Deviation of expanded cells", np.std(cells_s))
print("Average runtime ", np.mean(times_s))
print("Total runtime ", np.sum(times_s))

print("\n--------------------------------------------\n")

print("Forward A Star, greater tie breaker")
print("Average expanded cells ", np.mean(cells_g))
print("Standard Deviation of expanded cells", np.std(cells_g))
print("Average runtime ", np.mean(times_g))
print("Total runtime ", np.sum(times_g))
'''