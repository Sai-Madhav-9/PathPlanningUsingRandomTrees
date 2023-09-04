
import numpy as np

from src.rrt.rrt_star import RRTStar
from src.search_space.search_space import SearchSpace
from src.utilities.plotting import Plot
X_dimensions = np.array([(0, 300), (0, 300), (0, 10)])

# Room 1
room1_origin = (50, 50, 0)
room1_dims = (100, 100, 5)
room1_coords = np.array([    (room1_origin[0], room1_origin[1], room1_origin[2], room1_origin[0]+room1_dims[0], room1_origin[1]+room1_dims[1], room1_origin[2]+room1_dims[2])
])

# Room 2
room2_origin = (200, 50, 0)
room2_dims = (100, 100, 5)
room2_coords = np.array([    (room2_origin[0], room2_origin[1], room2_origin[2], room2_origin[0]+room2_dims[0], room2_origin[1]+room2_dims[1], room2_origin[2]+room2_dims[2])
])

# Room 3
room3_origin = (125, 175, 0)
room3_dims = (50, 100, 5)
room3_coords = np.array([    (room3_origin[0], room3_origin[1], room3_origin[2], room3_origin[0]+room3_dims[0], room3_origin[1]+room3_dims[1], room3_origin[2]+room3_dims[2])
])

# Combine all room coordinates
Obstacles = np.vstack([room1_coords, room2_coords, room3_coords])

# Starting and goal positions
x_init = (25, 25, 0)
x_goal = (275, 275, 0)

Q = np.array([(8, 4)])  # length of tree edges
r = 1  # length of smallest edge to check for intersection with obstacles
max_samples = 1024  # max number of samples to take before timing out
rewire_count = 32  # optional, number of nearby branches to rewire
prc = 0.1  # probability of checking for a connection to goal

# create Search Space
X = SearchSpace(X_dimensions, Obstacles)

# create rrt_search
rrt = RRTStar(X, Q, x_init, x_goal, max_samples, r, prc, rewire_count)
path = rrt.rrt_star()

# plot
plot = Plot("rrt_star_3d")
plot.plot_tree(X, rrt.trees)
if path is not None:
    plot.plot_path(X, path)
plot.plot_obstacles(X, Obstacles)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_goal)
plot.draw(auto_open=True)
