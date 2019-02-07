# Pursuit and Evasion

## Purpose
This library is supposed to make turtlebot wander around in two modes (pursuit and evation) without bumping into anything, by using the LaserScan message obtained from Asus Xtion Pro.

## Pre-requisites

### Hardware requirement
- A kobuki turtlebot base
- An Asus Xtion Pro
- A controller (prefered logitach)
- A device with minimum 3 usb ports

### Software requirement
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

## Execution

1. clone this repo into the source directory of your catkin workspace (e.g. catkin_ws/src)
    ```bash
    # under catkin_ws/src folder
    mkdir comp1
    git clone https://github.com/CMPUT412W19Team6/Competition1.git comp1
    ```
  
2. run catkin_make and source the setup.bash
    ```bash
    cd ..
    catkin_make
    source ./devel/setup.bash
    ```
    
3. connect your your kobuki base, Asus Xtion Pro and controller.

4. power up the kobuki base

5. start the library
    ```bash
    roslaunch comp1 multi_robot_simulation.launch
    ```

6. Start the turtlebot
    > note:  you have to press B to stop the robot when `switching betwwen different modes`
    - to start `Evade`  : press A on the controller
    - to start `Persuit`: press X on the contorller
    
## Concepts & Code

### Overview
***insert picture of state machine here

### Evade

### Persuit
