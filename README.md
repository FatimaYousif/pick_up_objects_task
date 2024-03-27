# Pick up objects task


In this Hands-on Planning lab attached with its package deliverable and repository [here](https://github.com/FatimaYousif/HOP_Lab_1) (which will be maintained for future work also), the first online path planning pipeline for Turtlebot3 was further developed. Having included the following classes as StateValidityChecker, Planner (RRT with smoothing) , and compute_path(), and move_to_point() functions alongside the OnlinePlanner class for online path planning and controlling a robot using ROS with Key functionalities including: Subscribing to various topics, implementing methods, providing velocity controller, implementing robot recovery (from obstacle avoidance) behaviour case to publishing markers for visualization in RViz. 
<br><br>

## Lab contributors & student ID:

| Name                    | ID       |
| -------------------     | -------- |
| Fatima Yousif Rustamani | u1992375 |
| Lisa Paul Magoti        | u1992463 |

## HOW TO RUN:

For running the normal task of turtlebot3_stage world in the launch file (pick_up_objects_task.launch), use the following command:

```bash
roslaunch pick_up_objects_task pick_up_objects_task.launch
```

For running the extra bonus task of having included stage3 world (challenging), use the following command:

```bash
roslaunch pick_up_objects_task pick_up_objects_task2.launch
```

### 2. New terminal

```bash
rosrun pick_up_objects_task pickup_behaviors_node.py
```

## Behaviour Tree structure:

<img src="/images/RViz.png" alt="RViz Image" width="400"/>  

## Special (Extra) Functionality:

The following are the added generic functions for ensuring safe navigation of the robot and robot control besides the RRT or Planner class´s auxiliary functions.

### __in_map__(self, loc)

The purpose of this general in-line method in the StateValidityChecker class is to check whether a given location (specified as a list of indices [x, y]) is within the boundaries of the map -  where called and necessary. It returns True if the location is within the map boundaries and False otherwise.

Used in: __position_to_map__, __check_vicinity__, and __is_valid__ functions

### __check_vicinity__(self, cell, distance, checking_path=False)

Another general method within the StateValidityChecker class, this function checks the vicinity of a given cell within a certain distance. It iterates through the cells around the specified cell within the given distance and checks if any of them are obstacles. If checking_path is True, it considers a reduced distance to check for obstacles, for more frequent checks during path planning. It returns True if the vicinity is free from obstacles and False otherwise. 

Used in: __is_valid__ function

### __is_robot_in_obstacle__(self)

This method checks if the robot is stuck in an obstacle by utilizing the is_valid method from the StateValidityChecker (svc) object. It extracts the current position of the robot (current_pose) and checks if it's within an obstacle (i.e., not a valid position). If the current position is not valid, indicating it's stuck in an obstacle, the function returns True; otherwise, it returns False.

Used in: __plan__ function

### __recover__(self)

The recover method in the OnlinePlanner class handles recovery behavior when the robot is stuck in an obstacle. It sets the robot to move backward for a predefined recovery duration to try to get it out of the stuck situation.

Used in: __plan__ function

## Video - Visualization:

[![Video of the robot planning the path](https://img.youtube.com/vi/suB0sYJcoR4/maxresdefault.jpg)](https://youtu.be/gElyY12p7Go "Watch the video on YouTube")
