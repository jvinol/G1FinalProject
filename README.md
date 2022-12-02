# G1FinalProject

The link to our presentation can be found <ins>[here](https://docs.google.com/presentation/d/1lRZsxsvUOAIYw7n4Ee_mOIopluhwSt4a05aiQCqFNEw/edit#slide=id.g1a41337ac68_0_108)</ins>

## Objective
To map an environment using a depth camera and then visualize the entire environment in 3D.

## Reasoning
There are multiple ways to map and visualize an environment in 2d that are available to users using ROS. The most common being using a lidar sensor and the Nav2 toolbox to map the environment and then RViz to view it. However, while there are ways to visualize the environment in 3D through the use of either 3D Lidar or Depth cameras, there are not too many easy solutions to visualize a completed 3d map due to how computationally heavy the data can be. Some of the available solutions involve using octomap (documentation can be found <ins>[here](http://octomap.github.io/)</ins>

Our project proposes utilizing the PyVista package available in python to map and visualize the 3D environment. You can read more about the package <ins>[here](https://docs.pyvista.org/)</ins>

## Background
The data being mapped is known as a point cloud. A point cloud is a set of discrete data points in space, each having their own x, y, z coordinates as well as other parameters such as the rgb color and the intensity. Point cloud data is usually stored in a .pcd file 

## Setup
This project utilized an Oracle Virtual Machine running Ubuntu 20.04 Focal Fossa and ROS2 Foxy Fitzroy.

 - To install Ubuntu 20.04 on a Virtual Machine, follow the instructions <ins>[here](https://linuxhint.com/install_ubuntu_virtualbox_2004/)</ins>
 - To install ROS2 Foxy, follow the instructions <ins>[here](https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html)</ins>

In addition this project utilizes the slam_toolbox to give the user an option to map their environment in 2D and the teleop_twist keyboard to control the robot.

To install the slam_toolbox, type the following into the command line

<code>sudo apt install ros-foxy-slam-toolbox</code>

To run the slam_toolbox, the following command can be used

<code>ros2 launch slam_toolbox online_async_launch.py</code>




