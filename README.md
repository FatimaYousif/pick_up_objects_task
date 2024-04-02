# Pick up objects task

This repository consists the lab work on Behaviour trees for the Pick up objects task at hand to achieve. The project is detailed with the structure of the behaviour tree, its respective py_trees implementation, 
followed by the results for both the tasks a normal world task (turtlebot3_stage) alongside the challenging world task in (turtlebot3_stage_3), and visualization in RViz. 
<br>

## Lab contributors & student ID:
<table>
  <tr>
    <th>Name</th>
    <th>ID</th>
  </tr>
  <tr style="background-color: transparent;">
    <td>Fatima Yousif Rustamani</td>
    <td>u1992375</td>
  </tr>
  <tr style="background-color: transparent;">
    <td>Lisa Paul Magoti</td>
    <td>u1992463</td>
  </tr>
</table>

## HOW TO RUN:

For running the normal task of turtlebot3_stage world in the launch file (pick_up_objects_task.launch), use the following command:

```bash
roslaunch pick_up_objects_task pick_up_objects_task.launch
```

For running the extra bonus task of having included stage3 world (challenging), uncomment the following line in the turtlebot3_stage.launch launch file:

```bash
<arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_stage_3.world"/>
```

### New terminal

```bash
rosrun pick_up_objects_task pickup_behaviors_node.py
```

## Behaviour Tree :

<img src="/images/BT.png" alt="BT Image"/>  

## Behaviour Tree - explanation

To perform the pick and placing of 2 different objects (coke and bear) on its 5 possible yet different pick points, we implemented the following few additional classes except for the already given once including CheckObject, GetObject, LetObject alongside the given move behaviour.

1. SetPoint: This behavior sets the goal point for the robot to move to for picking or placing an object. It reads from the list of pick points or place points based on the number of points already visited and increments the counter for visited points.

2. MoveRobot: This behavior controls the movement of the robot to a specified goal position. It subscribes to the robot's odometry to get its current position and orientation and publishes a goal position to the `/move_base_simple/goal` topic for navigation.

3. Loop Termination: This composite behavior is a sequence that ensures the loop continues until either the number of objects collected (`objects_collected`) is less than the total number of objects to collect (len(place_points)) or the number of points visited (`points_visited`) is less than the total number of pick points (len(pick_points)).

4. Root: This is the main behavior tree's root node, which is also a sequence. It executes behaviors in the following order: loop termination, set point, move to pick, check object, get object, move to place, and let object. This sequence ensures that the robot repeatedly picks up and places objects until all objects are collected or all pick points are visited.

It uses blackboard variables to communicate state information between different behaviors and keep track of the progress of tasks.

## Video - Visualization:

#### Easy scenario - without obstacles - turtlebot3_stage_1.world
![Video](/images/BT_stage1.gif)

#### Challenging scenario - with obstacles - turtlebot3_stage_3.world
![Video](/images/BT_stage3.gif)

## Problems Found

We did not face any hard or big problem, however an easy-level problem was the understanding and managing Blackboard component in py_trees for implementation purposes.

## Conclusion

In conclusion, this lab proved to be a great learning resource to the main topic of behaviour trees especially for the pick and place tasks (object manipulation) beyond just the theoretical concept. The already provided individual components of the behaviour tree with further added behaviours showed how extensive it can be to use the behaviour trees with py_trees.
