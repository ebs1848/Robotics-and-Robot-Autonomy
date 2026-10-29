# Robotics-and-Robot-Autonomy

Robotics coursework in Python: control, navigation and path planning for simple simulated mobile robots. Each folder is self-contained and runs on its own.

| Folder | Topic |
| --- | --- |
| [`lane-control/`](lane-control) | Steering control of a bicycle-model robot for a lane change (Dummy, Bang-Bang and PID controllers) |
| [`bug-navigation/`](bug-navigation) | Bug0, Bug1, Bug2 and Tangent Bug planners with bump sensors and a lidar |
| [`graph-search/`](graph-search) | BFS, DFS, Dijkstra and A* over a visibility graph in configuration space |

## Requirements

Python 3 with NumPy 1.x and Matplotlib. `bug-navigation` also needs Shapely. NumPy 2 is not supported because the code calls `np.cross` on 2D vectors.
