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

## Execution

## Concepts & Code
