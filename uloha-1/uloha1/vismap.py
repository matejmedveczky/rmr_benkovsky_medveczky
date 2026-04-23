import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

grid = np.loadtxt("map.txt")

if grid is not None:
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Create 5-color map: UNKNOWN, FREE, OCCUPIED, BUFFER, FLOODED
    colors = ['darkred', 'yellow', 'darkgreen', 'lightgreen', 'lightblue']
    cmap = ListedColormap(colors)
    
    # Map flood values (≥4) to color index 4
    display_grid = grid.copy()
    display_grid[display_grid >= 4] = 4
    
    ax.imshow(display_grid, cmap=cmap, vmin=0, vmax=4, origin='lower')
    
    # Add text for flood values
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            if grid[r, c] >= 4 and grid[r, c] % 5 == 0:
                ax.text(c, r, str(int(grid[r, c])), 
                       ha="center", va="center", 
                       color="blue", fontsize=6)
    
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    plt.tight_layout()
    plt.show()
else:
    print("Error loading map.txt")