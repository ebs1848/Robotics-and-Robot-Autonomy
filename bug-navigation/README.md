# Bug navigation

Bug0, Bug1, Bug2 and Tangent Bug path planners for a mobile robot (`bug_planners.py`), run in a simulation through `bug_robot.py`. Bug0 to Bug2 use bump sensors, Tangent Bug uses a lidar.

## Planners

The planner is selected with the `planner` argument of `Robot(...)` in `bug_robot.py`: 0 for Bug0, 1 for Bug1 and 2 for Bug2, which use the five bump sensors, and 3 for Tangent Bug, which uses a 360-beam lidar (30 cm range). Tangent Bug drives straight at the goal while the way is free, heads for the obstacle edge with the shortest estimated path when something blocks it, follows the boundary at a local minimum and leaves it as soon as the lidar sees a point closer to the goal than anything seen on the followed obstacle. If it comes back to the same point without a way out, it reports that no path exists.

## Run

```
python bug_robot.py
```
