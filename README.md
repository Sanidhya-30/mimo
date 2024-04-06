# Differential Drive Robot Navigation Project

## Overview
This project involves programming a differential drive robot using ROS (Robot Operating System) for navigation. The robot is equipped with a laser scanner and IMU sensor for localization and obstacle avoidance. The goal is to navigate the robot through a predefined set of waypoints, stopping at each waypoint, aligning its yaw angle, and then moving to the next waypoint. Once the last waypoint is reached, the robot spins on its axis for 3 rotations in a clockwise direction.

## Features
- Utilizes ROS (Robot Operating System) for robot control and navigation.
- Implements differential drive kinematics for robot movement.
- Uses laser scanner data for obstacle detection and avoidance.
- Integrates IMU sensor data for robot orientation.
- Follows a set of predefined waypoints, stopping at each and aligning yaw angle.
- Implements spinning motion for the robot upon reaching the last waypoint.

## Files Included
- `move.py`: Python script for robot navigation through waypoints and spinning.
- `reading_laser.py`: Python script for reading and filtering laser scan data.
- `README.md`: This file, providing an overview of the project.

## Requirements
- ROS (Robot Operating System) - Noetic version.
- Python 2.7 or later.
- Robot equipped with laser scanner and IMU sensor.
- Compatible hardware for running ROS nodes.

## Usage
1. Clone the repository:
   ```bash
   git clone https://github.com/sanidhya-30/mimo.git

2. Catkin_make:
   ```bash
   cd ~/mimo && catkin_make

1. Launch Scripts:
   ```bash
   roslaunch bot_control move.launch
