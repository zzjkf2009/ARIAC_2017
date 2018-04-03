# Overview
The objective of this project is to make a Task-level planning to test the agility of industrial robot system, with the goal of enabling industrial robots on the shop floors to be more productive, more autonomous and be more responsive to the needs of shop floor workers. The Planning is been designed and tested for the Agile Robotics for Industrial Automation Competition [ARIAC](http://gazebosim.org/ariac). Gazebo provides the simulation infrastructure and the environment. In order to test the agility of the planner, despite picking and placing the desired part to a correct pose, it also may encounter several scenarios, like dropping the part, get higher priority order during the process, pick form conveyor belt etc.
![ARIAC](https://github.com/move_arm/ARIAC.png)

## Package
This is a costumed package developed by our group which contains the level-planning and control using MoveIt! to control the industrial robot (UR10 form Universal robot) and a simple task-level control to solving the (qual2a)[https://bitbucket.org/osrf/ariac/wiki/2017/qualifiers/qual2_scenarios]. In this qualifier, our planner need to deal with problem of dropping part: One of the piston rod parts is picked up from the bin, it will be dropped back into the bin. We have to retrieve this part in order to complete the order. In addition, when one of the gear parts is placed over a tray, it will be dropped into the tray in an incorrect position. we need to re-position this part to complete the kit with the parts in the correct poses.

## Pre-request
- Gazebo
- ROS (kinetic)
- ARIAC(2017) (installation:ariac wiki)[http://wiki.ros.org/ariac/Tutorials/SystemSetup]
- MoveIt!

## Build
```
cd catkin_ws
source devel/setup.bash
catkin_make
```

## Start the competition and qualifier
### Start ARIAC Environment
```
rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual2a.yaml ~/ENPM809_ws/src/move_arm/config/qual2_config.yaml
```

### Start MoveIt
```
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true
```

### Start move_arm service node
```
rosrun move_arm move_srv
```
### Start the move_arm excuation Node
```
rosrun move_arm move_test
```
### command line to move the arm by pose
To move the arm to pick up part, call service like the following,
to place(release) the part, turn it to mode 2
```
rosservice call /move_arm/toPose "pose:
  position:
    x: -0.2
    y: 0.33
    z: 0.745
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
mode: 1"
```
## Result
Our package successfully pass the qualifier 2 and finish all the desired tasks during a reasonable time. There is the link of the result (Video)[https://www.youtube.com/watch?v=EdZv5IVYHCU&feature=youtu.be]
