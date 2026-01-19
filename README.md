# Fenrir
An Autonomous differential drive project in ROS 2.

The goal of this project is a differential-drive robot with autonomous navigation, such that goals can be input through the ROS Nav2 package and the robot executes based on its behaviour tree configuration. In the future, goals would be input using natural language and processed through an LLM.
Fenrir, the name I've given this bot (I'm a fan of Norse Mythology), utilizes both Lidar and rgbd cameras for precise SLAM mapping, and a YOLO model for visualization. 
This project started from a udemy tutorial by Antonio Brandi. It has then since grown with many additions and optimizations. I encourage beginners to check out his courses. This is purely created for research and learning and is not a commercial product.

Each package serves a distinct purpose, which can be inferred by name. For the sake of eliminating ambiguity, it shall be explained below. For Installation instructions, [Click here (or find below)](#installation).
#

<a name=packages></a>
### Packages
**Robot description (bumperbot_description)**
This package contains the urdf files for the robot, alongsides the gazebo configurations for the sensors. It also contains the STL files for rendering in both rviz and gazebo. The 'Worlds' directory contains directories for prebuilt physical settings. Currently, there's a wharehouse setting, an empty setting and a setting containing only some traffic cones (This makes slam run easier on limited compute). 
This package also contains some rviz configurations, which aren't of utmost importance but saves everyone a few minutes each time Rviz is launched.
This package, like every other package in the ws, contains launch files. One for gazebo, and the other for Rviz. The display launch file launches gazebo, the other launches Rviz. Changing which World (or setting) is launched must be done manually for now, but will be changed to allow easy switching through launch arguments in the near future.

**Examples (bumperbot_examples)**:
These are example scripts for people who want to check simplified examples of many of the utilities of ros utilized in this package.
If you're starting your ros journey, feel free to take a look at this package first.

**Controller (bumperbot_controller)**:
This package handles translating the commands from the input to the velocities of the wheel based on the dynamics of the bot using twist stamped messages (A communication structure in ros2). It also implements a 'noisy controller' that adds Gaussian noise to simulate real sensor noise. No feed back control has been implemented yet. 

**Local Localization (bumperbot_localization)**:
This package deals with **local localization**. Local localization refers to estimating the position of a bot based on integrating its movements. In the package, a stochastic approach is used using an Extended Kalman Filter. This outputs the calculated position as a probability (mean), and also gives the certainity of its calculation (variance).

**Visualization (bumperbot_visualization)**:
This package handles the vision model, YOLO11, which runs object detection. (No segmentation). Currently, I'm thinking of creating another model through transfer learning from the yolo model for better detection of objects more likely to be seen by Fenrir, or upgrading to the m/l models.

**Sensor Reframing and Bridging (bumperbot_bridges)**:
For the sake of Rtabmap (a SLAM package), some topics from the lidar and camera sensors had to have their header's frame ID changed and then republished. Some had to be bridged from Gazebo. Sensors are configured in gazebo and then 'bridged' to ros using ros_gz_bridge, else ROS wouldn't have access to those topics. It used to be in visualization, but sometimes I had to test slam without running the visualization package, so I created a new package entirely. I assume there are better ways of solving this problem, which I receive with open arms.

**SLAM (bumperbot_slam)**:
SLAM (Simultaneous Localization and Mapping) refers to 2 things. The first is Mapping. That is, building a map of the environment from sensor scans. Lidars are typically the primary sensors used for this task, but rgbd (d for depth) camera sensors are also used. In this project, both have been combined to create maps. Localization refers to estimating where the robot is with the map being built as a reference. For example, imagine Fenrir was dropped in a bedroom. Mapping answers the question 'What is the geometry of this room (including objects)?' while localization answers 'Where am I within this geometry?'.
**Rtabmap** is an Open source package that works well with ROS2. It handles all the terrorizing math and physics and provides a clean API which was used in this project.

**Navigation (bumperbot_nav)**:
This takes the map from the slam package and actually navigates through the map when given a task (or 'Goal'). It uses Behaviour Trees (BTs) as well as other sensor data for decision-making. Uses Ros2's 'Nav2' package internally. Currently still in development.
#

<a name=installation></a>
### Installation Guide
**Prerequisites**
1) This was created and tested on an Ubuntu 24.04 OS, using ROS2 jazzy. Using another version of ROS2 or a different operating system may cause errors I'm not familiar with. If so, feel free to report as an Issue on this repo.
2) Rtabmap and rtabmap_ros are essential packages I didn't include here and must be installed separately. They should be moved into the src/ directory after installation. It is not necessary if navigation and slam aren't needed. Installation instructions can be found [here](https://github.com/introlab/rtabmap/wiki/installation).
3) Running gazebo with lidars consumes a lot of compute. If running with limited compute (no GPU like me), a distributed computing solution may work, with a raspberry pi or Jetson Nano, or a second pc. If none are available, manually disabling the Lidar sensor is your best bet. Unfortunately that would mean SLAM and navigation wouldnt work. 

**Installation**
1) Create a workspace (directory/folder) for the codebase. 
```bash
mkdir <workspace_name> 
cd <workspace_name>
#Replace <workspace_name> with a name of your choice. Eg. bumperbot_ws
```
#

2) Clone this repo in that directory.
```bash
git clone https://github.com/kogodemilade/Fenrir--Autonomous-Diff-Drive-Bot/
```
#

3) Install python and ros dependencies.
```bash
#Python dependencies
pip install -r requirements.txt
#Missing ros packages
rosdep install --from-paths src --ignore-src -r -y
```
#
4) Build repository.
```bash
colcon build
```
#

### Usage
The first step whenever opening a terminal or a terminal tab is to source ros and source the project in question.
```bash
#Source ros
. /opt/ros/jazzy/setup.bash

#source current project (bumperbot_ws, replace with your directory file path)
cd ~/path/to/project && . install/setup.bash
```
In practice, sourcing the current project is usually enough, although I've found that sometimes sourcing both fix weird errors once in a while.

I suggest creating an [alias](https://askubuntu.com/questions/17536/how-do-i-create-a-permanent-bash-alias) since this commands are used very frquently.

All packages must run in separate terminal tabs, so I recommend Terminator, which allows splitting tabs using Ctrl+shift+E (horizontal split) and Ctrl+shift+O (Vertical splits). This makes it easy to monitor multiple processes in prallel.
All packages can be launched using
```bash 
ros2 launch bumperbot_<pkg> app.launch.py
```
For simulations and visualization of some of the afore-mentioned topics, the description, control and local localization packages are enough. 
```bash
ros2 launch bumperbot_description gazebo.launch.py
#New tab (Ctrl+Shift+E on terminator)
ros2 launch bumperbot_controller controller.launch.py
#New tab 
ros2 launch bumperbot_description display.launch.py
#New tab
ros2 launch bumperbot_localization local_localization.launch.py
```
For vision or SLAM, the bridges package **must** be launched first.
It is advised to launch the packages in this order- the gazebo launch file, then the control launch file- for reasons relating to synchronization. All other packages can be launched in arbitrary order depending on what is required.


