'''
import Maze_Generator as mazeGen
import Repeated_Forward_Astar as fAstar
import Repeated_Backward_Astar as bAstar
import numpy as np

mazeDim = 51
p = .3

cells_f = [] 
times_f = []
cells_b = [] 
times_b = []

for i in range(0, 50):
    OG_maze = mazeGen.create_maze(mazeDim, p, False)
    expanded_cells_f, time_elapsed_f = fAstar.astar_forward(OG_maze, mazeDim, fAstar.tie_breaker_greater, False)
    cells_f.append(expanded_cells_f)
    times_f.append(time_elapsed_f)
    expanded_cells_b, time_elapsed_b = bAstar.astar_backward(OG_maze, mazeDim, False)
    cells_b.append(expanded_cells_b)
    times_b.append(time_elapsed_b)

print("Forward A Star")
print("Average expanded cells ", np.mean(cells_f))
print("Standard Deviation of expanded cells", np.std(cells_f))
print("Average runtime ", np.mean(times_f))
print("Total runtime ", np.sum(times_f))

print("\n--------------------------------------------\n")

print("Backward A Star")
print("Average expanded cells ", np.mean(cells_b))
print("Standard Deviation of expanded cells", np.std(cells_b))
print("Average runtime ", np.mean(times_b))
print("Total runtime ", np.sum(times_b))
'''