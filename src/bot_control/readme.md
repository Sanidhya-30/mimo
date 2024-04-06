# bot_control Package

This package provides functionality for controlling a robot, specifically focusing on reading laser scan data, filtering it, and visualizing the filtered scan data using RViz.

## Installation

1. Make sure you have ROS (Robot Operating System) installed. This package is developed and tested on ROS Noetic.

2. Create a ROS workspace (if you haven't already):
   ```bash
   mkdir -p ~/[your_name]_ws/src
   cd ~/[your_name]_ws/src
   catkin_init_workspace

3. Clone the repository:
   ```bash
   git clone https://github.com/sanidhya-30/mimo.git

4. Catkin_make:
   ```bash
   cd ~/mimo && catkin_make

5. Launch Scripts:
   ```bash
   roslaunch bot_control move.launch
