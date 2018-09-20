# Overview
The objective of this competition is to make a Task-level planning to test the agility of industrial robot system, with the goal of enabling industrial robots on the shop floors to be more productive, more autonomous and be more responsive to the needs of shop floor workers. The Planning is been designed and tested for the Agile Robotics for Industrial Automation Competition [ARIAC](http://gazebosim.org/ariac) hosted by NIST (national Institute of Standard and Technology). Gazebo provides the simulation infrastructure and the environment. In order to test the agility of the planner, despite picking and placing the desired part to a correct pose, it also may encounter several scenarios, like dropping the part, get higher priority order during the process, pick form conveyor belt etc.
![ARIAC](https://github.com/zzjkf2009/move_arm/blob/master/ARIAC.png)

## Qualifiers
This is a costumed package developed by our group which contains the task-level planning and control using MoveIt! to control the industrial robot (UR10 form Universal robot).  Several scenarios may happened during the packing process include failing suction grippers, notification of faulty parts, and reception of high-priority orders.
 - [qual1](https://bitbucket.org/osrf/ariac/wiki/2017/qualifiers/qual1).In the qualifier, our planner will deal with the *Baseline Kit Building* by reading the orders contains specif required parts and their desired place locations on the tray and finding the parts' pick locations (either on the bin or belt). Known the desire pick-place location. The arm be controlled with [MoveIt](https://moveit.ros.org/), a state of art software for mobile manipulation, to do the grasping (here we use a vacuum) and placing, meanwhile avoiding the obstacles in the work cell.
 - [qual2](https://bitbucket.org/osrf/ariac/wiki/2017/qualifiers/qual2_scenarios). In this qualifier, our planner need to deal with problem of dropping part: One of the piston rod parts is picked up from the bin, it will be dropped back into the bin. We have to retrieve this part in order to complete the order. In addition, when one of the gear parts is placed over a tray, it will be dropped into the tray in an incorrect position. we need to re-position this part to complete the kit with the parts in the correct poses.
 - [qual3](https://bitbucket.org/osrf/ariac/wiki/2017/qualifiers/qual3). In this qualifier, our planner need to deal with *In-process Kit Change*. A new order with higher priority will be announced part-way into the completion of the first order. To complete the second order, we can either choose to remove the parts already in the kit tray(s), switch to an available empty tray, or submit the first order partially complete to receive an empty tray when the AGV returns. After the second order is complete, the first order is to be resumed.
 - [qual4](https://bitbucket.org/osrf/ariac/wiki/2017/qualifiers/qual3). In this qualifier, our planner need to deal with *Faulty part*. The quality control sensors above the tray will publish the pose of faulty parts that they see on the tray. The faulty part needs to be removed from the tray (AGV) and replaced by a new required part.   

## Challenges and Solution
### The challenges of this competition contains the following parts:
- 1. Localize the part either on the stationary bins or on the conveyor belt. The arm need to know where to pick and place the part. The intelligent camera (a hacked camera in simulation) could detect the part location relative to its frame. However cost of each camera is high, we want to minimize the total cost for this system to keep it as low as possible.
- 2. Kit order queue management. During the packing process, we may encounter the situation of dropping the part, picking the faulty part and replace it with a new one, the higher priority order interrupt the processing order etc.
- 3. The arm control. How arm controlling and motion planning for the arm to gently grasp and place the parts from/to desired location and meanwhile avoid colliding with obstacles.
### Solutions
In order to overcome those challenges, a task-level task planner is needed handle every possible scenario. We first build a knowledge representation graph including task decompositions, agent modeling, world modeling, variable modeling etc. Then create the architecture of the system as showed in the following image. Finally, a task-level planner is designed for the pick-and-place action.
- ![UML](https://github.com/zzjkf2009/move_arm/blob/master/UML.jpg)
We divide our work into three part: localization (perception on the bin/belt), order management, arm planning/control. And each of our team member take charge one part by creating its own ROS package. Necessary info will communicate between each package through topics and services. How the package communicate with each other is showed as figure below:
- ![UML](https://github.com/zzjkf2009/move_arm/blob/master/Architecture.png)

## Pre-request
- Gazebo
- ROS (kinetic)
- ARIAC(2017) [installation:ariac wiki](http://wiki.ros.org/ariac/Tutorials/SystemSetup)
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
rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual3a.yaml ~/ENPM809_ws/src/move_arm/config/qual3_config.yaml
```

### Start MoveIt
```
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true
```

### Start move_arm service node
```
rosrun move_arm move_srv
```
### Start part_detect
```
rosrun part_perception detect_part
```
### Start the move_arm execution Node
```
rosrun move_arm move_orderManager
```
### command line to move the arm by pose
To move the arm to pick up part, call service like the following,
to place(release) the part, turn it to mode 2
```
rosservice call /move_arm/toPose "pose:
  position:
    x: -0.4
    y: -0.635
    z: 0.74
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
mode: 1"
```
## Result
Our package successfully pass all the qualifiers and finish all the desired tasks during a reasonable time, which is the best team among the six teams. There are the links of the result
- [![Qual2a](https://img.youtube.com/vi/EdZv5IVYHCU/0.jpg)](https://www.youtube.com/watch?v=EdZv5IVYHCU)
- [![Qual2b](https://img.youtube.com/vi/EmTPnTIXFfU/0.jpg)](https://www.youtube.com/watch?v=EmTPnTIXFfU)
- [![Qual3a](https://img.youtube.com/vi/UthBsMiXU0U/0.jpg)](https://www.youtube.com/watch?v=UthBsMiXU0U)
- [![Qual3b](https://img.youtube.com/vi/gtExci9a5ag/0.jpg)](https://www.youtube.com/watch?v=gtExci9a5ag)
