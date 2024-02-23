import random as RandomValue
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
from matplotlib.colors import ListedColormap

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

    cmap = ListedColormap(['black', 'white', 'green', 'red'])
    
    plt.figure(figsize=(5, 5))
    sns.heatmap(maze, linewidths=0.1, linecolor="gray", square=True, cbar=False, xticklabels=False,
                yticklabels=False, cmap=cmap, vmin=0, vmax=3)  # Use the custom colormap here
    plt.show()
