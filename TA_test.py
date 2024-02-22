import Maze_Generator as mazeGen
import Repeated_Forward_Astar as fAstar
import Repeated_Backward_Astar as bAstar
import Adaptive_Astar as aAstar
import numpy as np

mazeDim = 51
p = .3

OG_maze = mazeGen.create_maze(mazeDim, p, True)


print("Forward A star, smaller tie breaker")
fAstar.astar_forward(OG_maze, mazeDim, fAstar.tie_breaker_smaller, True)

print("--------------------------------------------")

print("Forward A star, greater tie breaker")
fAstar.astar_forward(OG_maze, mazeDim, fAstar.tie_breaker_greater, True)

print("--------------------------------------------")

print("Backward A star")
bAstar.astar_backward(OG_maze, mazeDim, True)

print("--------------------------------------------")

print("Adaptive A star")
aAstar.astar_adaptive(OG_maze, mazeDim, True)