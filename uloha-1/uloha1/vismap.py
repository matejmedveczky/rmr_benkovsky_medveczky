from math import e

import numpy as np
import matplotlib.pyplot as plt
plt.xlabel("X")
plt.ylabel("Y")
grid = np.loadtxt("map.txt")
grid = np.flipud(grid)   # flip vertically
if grid is not None:
    plt.imshow(grid, cmap="RdYlGn")
    plt.show()
else:
    print("Error loading map.txt")