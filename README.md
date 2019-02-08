# Pursuit and Evasion
**Description:** In this competition, two teams face off against each other where one team's robot tries to evade the other team's pursuing robot.

## 1. Purpose
This repository contains code for both pursuing and evading. Pursuing is done by utilizing the robot's LaserScan data to detect the closest object that is within a configurable field of vision and then following that object. The evader code makes the robot move away from obstacles detected by the camera or backup and then move away from obstacles detected by the bumper sensor. Morever, the evader also changes moving direction randomly after going straight for some configurable amount of time.

## 2. Pre-requisites

### 2.1 Hardware requirement
- A kobuki turtlebot base
- An Asus Xtion Pro
- A controller (prefered logitach)
- A device with minimum 3 usb ports

### 2.2 Software requirement
- ROS kinetic ([Guide here](http://wiki.ros.org/kinetic/Installation/Ubuntu))

- Turtlebot  packages ([Guide here](http://wiki.ros.org/action/show/Robots/TurtleBot?action=show&redirect=TurtleBot))
  ```bash
  sudo apt-get install ros-kinetic-turtlebot
  sudo apt-get install ros-kinetic-turtlebot-apps
  sudo apt-get install ros-kinetic-turtlebot-interactions
  sudo apt-get install ros-kinetic-turtlebot-simulator
  ```
  
- Kobuki ROS packages ([Guide here](https://wiki.ros.org/kobuki/Tutorials/Installation))
  ```bash
  sudo apt-get install ros-kinetic-kobuki
  sudo apt-get install ros-kinetic-kobuki-core
  ```
  
- Upgrade camera packages
  ```bash
  sudo apt-get install ros-kinetic-openni2-camera
  sudo apt-get install ros-kinetic-openni2-launch
  ```
  
- ROS smach ([Guide here](http://wiki.ros.org/smach))

- rviz ([Link here](http://wiki.ros.org/rviz))

## 3. Execution

### 3.1 Quickstart

1. Clone this repo into the source directory of your catkin workspace (e.g. catkin_ws/src)
    ```bash
    # under catkin_ws/src folder
    mkdir comp1
    git clone https://github.com/CMPUT412W19Team6/Competition1.git comp1
    ```
  
2. Run catkin_make and source the setup.bash
    ```bash
    cd ..
    catkin_make
    source ./devel/setup.bash
    ```
    
3. Connect your your kobuki base, Asus Xtion Pro and controller.

4. Power up the kobuki base

5. Start the library
    ```bash
    roslaunch comp1 multi_robot_simulation.launch
    ```

6. Start the turtlebot
    > note:  you have to press B to stop the robot when `switching betwwen different modes`
    - to start `Evade`  : press A on the controller
    - to start `Persuit`: press X on the contorller
    
7. To stop the turtlebot, press B on the controller

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


    
## 4. Concepts & Code

### Overview
  - state machine:
  ![statemachine](https://github.com/CMPUT412W19Team6/Competition1/blob/master/statemachine.png?s=200)

### Evade

  `Concept`: 
  
    1. Move straight until the camera found anything that's within a 1.1 meters.
    
    2. Turn for 3 seconds. If range is less than 0.7 meter, turn with a higher anguler speed. 
       Then check if the anything's winth `range`.
      > if yes, go to step 2
      > if no, go to step 1
      
    3. In case of a bump:
      3.1 move back 0.15 meter
      3.2 turn left
      3.3 move 0.4 meter
      3.4 
        > if no more bump, turn back and go to step 1
        > if another bump, go to step 3.1
### Persuit
