import random as RandomValue
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

def create_maze(dim, p, visualize):
    numCol = dim
    numRow = dim

    initialMaze = np.ones((dim, dim))  

    for i in range(numRow):
        for j in range(numCol):
            if (RandomValue.uniform(1,0) < p):  
                initialMaze[i][j] = 0

    initialMaze[0][0] = 2
    initialMaze[dim-1][dim-1] = 3

    if visualize:
        visualise_maze(initialMaze)

    return initialMaze


def visualise_maze(maze):  

    plt.figure(1, figsize=(5, 5))
    ax = sns.heatmap(maze, linewidths=0.1, linecolor="gray", square=True, cbar=False, xticklabels=False,
                     yticklabels=False, cmap=sns.diverging_palette(10, 220, n=200))
    plt.show()
