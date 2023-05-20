# RoboBreizh@Home ROS2 (Humble) Navigation 2 Stack

## Summary
This repository contains a ROS2-Nav2 Stack package for navigating Pepper Robot in the Gazebo simulation environment. Some resources credited to [Automatic Addison](https://automaticaddison.com/) and [ros-planning](https://github.com/ros-planning/navigation2). This package utilises Planar-Driven plugin to drive Pepper in X,Y,Z axis in Gazebo simulation

<p align=""><img src="Example/pepper_nav2.gif" width="700"\></p>


## Prerequisite 

1. ROS2 Humble installed on Ubuntu Linux 22.04 ie. ROS2 Foxy for Ubuntu Linux 20.04
2. Installed ROS2 Control and Gazebo 11, please refer to this [Gazebo ROS2 repository](https://github.com/RoboBreizh-RoboCup-Home/gazebo_ros2_pepper) for more details.
3. And follow the steps below for installing required packages for Nav2 Stack.

## 1. Installation

```
sudo apt install ros-<ros distro>-navigation2
sudo apt install ros-<ros distro>-nav2-bringup
sudo apt install ros-<ros distro>-turtlebot3*
sudo apt install ros-<ros distro>-slam-toolbox
sudo apt-get install ros-<ros-distro>-rqt-robot-steering
mkdir ~/nav2_ws/src
cd ~/nav2_ws/src ie. Go to your ROS2 workspace
git clone https://github.com/ros-planning/navigation2.git --branch <ros-distro> ie. modify ros-distro
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros-distro>
colcon build --symlink-install
```
You can open the bashrc file, export turtlebot if you want to spawn turtlebot for demo

```
source ~/nav2_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/foxy/share/turtlebot3_gazebo/models
source /usr/share/gazebo/setup.sh
```
Then, build the package again
```
colcon build --symlink-install
```

## 2. Git LFS clone

1. Since this repository contains LFS file, please use Git LFS clone to any directory you want to store this files.
```
git lfs clone https://github.com/RoboBreizh-RoboCup-Home/gazebo_ros2_navigation.git
```
2. Copy all files to nav2_ws/src/navigation2/nav2_bringup


## 3. Change local path (.world and .sdf)

1. Open ./worlds/completed_arena.world and replace all dae uri (total 12 lines) with your local directory containing the dae files located in the dae folder in the same package.\
Example:
```
  <geometry>
    <mesh>
      <uri>/home/crossing/nav2_ws/src/navigation2/nav2_bringup/dae/Arena_all.dae</uri> ie. change this line 
      <scale>1 1 1</scale>
    </mesh>
  </geometry>
```

## 4. Launch Navigation Stack 

RVIZ and Gazebo will be launched after executing the launch command. The Arena Map was pre-built by using Turtle-bot3 with 360 degree of lidar sensor.  
```
cd ~/nav2_ws/
colcon build --packages-select nav2_bringup
ros2 launch nav2_bringup pepper_launcher.py headless:=False
```

## 5. Launch Turtlebot 3 with SLAM (Draw Map)

You can use rqt_robot_steering tool to move the robot around the area in Gazebo for map drawing.
```
ros2 launch nav2_bringup draw_map_launcher.py slam:=True headless:=False
ros2 run rqt_robot_steering rqt_robot_steering --force-discover
```
Open new termianl
```
ros2 launch nav2_bringup map_saver.launch.py
```
Open another new termianl
```
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: custom_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```
You should be able to get custom_map.pgm and custom_map.yaml files in your nav2_ws workspace. 
Copy the map.yaml and map.pgm files to the map folder in the nav2_bringup package if you want to launch your custom map.

