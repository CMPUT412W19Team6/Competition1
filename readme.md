# CMPUT 412 W19 Competition 1
**Description:** In this competition, two teams face off against each other where one team's robot tries to evade the other team's pursuing robot.

## 1. Purpose
This repository contains code for both pursuing and evading. Pursuing is done by utilizing the robot's LaserScan data to detect the closest object that is within a configurable field of vision and then following that object. The evader code makes the robot move away from obstacles detected by the camera or backup and then move away from obstacles detected by the bumper sensor. Morever, the evader also changes moving direction randomly after going straight for some configurable amount of time.

## 2. Pre-Requisites

First make sure ROS Kinetic (Desktop Full) is installed properly by following [these instructions](http://wiki.ros.org/kinetic/Installation). Then install the ROS Kinetic version of the following packages:

- turtlebot
- turtlebot_apps
- turtlebot_interactions
- turtlebot_simulator
- kobuki
- kobuki_core
- openni2_camera
- openni2_launch

In Ubuntu, following should install these packages:


```
> sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki ros-kinetic-kobuki-core ros-kinetic-openni2-camera ros-kinetic-openni2-launch
```

## 3. Execution

Create a workspace if one has not been created already by following [these instructions](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). Download or clone this repository to ```<workspace_path>/src``` (replace ```workspace_path``` with the path of your workspace). Assuming the workspace directory is located at ```~/catkin_ws```, open a terminal and execute the following commands:

```
> cd ~/catkin_ws
> source devel/setup.bash
> catkin_make
```

After the above commands, in the same terminal execute the following to run the simulation:

```
> roslaunch comp1 multi_robot_simulation.launch
```

This will bring up a simulated world in the Gazebo simulator with one evading Turtlebot and one that pursues that evader. (multi Turtlebot simulation was taken from [here](https://github.com/scheideman/Multiple_turtlebots))

To run the code on the actual robot, connect the Kobuki base, the camera and a joypad to the computer and run the following in the same terminal:

```
> roslaunch comp1 comp1.launch
```

If the ```start_mode``` parameter is set to ```none``` or not modified, pressing ```B``` will make the robot evade, ```A``` will make it stop and ```X``` will make the robot pursue.

### 3.1 Parameters

**~start_mode (string, default: "none")**

Which mode the robot will be in at startup. ```none``` will make the robot wait for a button press to either start evading or pursuing. ```pursuer``` will start the robot in pursuer mode while ```evader``` will make the robot evade after startup.

**~pursuer_goal_z (double, default: 0.86)**

The goal distance (in meters) to keep between the evader and the pursuer

**~pursuer_z_threshold (double, default: 0.02)**

How far away from the goal distance (in meters) before the pursuer reacts

**~pursuer_x_threshold (double, default: 4.0)**

How far away (in degrees) from being centered (x displacement) on the person before the robot reacts

**~pursuer_field_of_view (double, default: 41.0)**

How wide (in degrees) should the pursuer's vision be in which to look for evader (0 <= pursuer_field_of_view <= 58)

**~pursuer_max_angular_speed (double, default: 0.8)**

The maximum rotation speed (in radians per second)

**~pursuer_max_linear_speed (double, default: 0.6)**

The max linear speed (in meters per second)

**~pursuer_ramped_rate (double, default: 0.3)**

Rate for ramped velocity change
