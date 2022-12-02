# G1FinalProject

The link to our presentation can be found <ins>[here](https://docs.google.com/presentation/d/1lRZsxsvUOAIYw7n4Ee_mOIopluhwSt4a05aiQCqFNEw/edit#slide=id.g1a41337ac68_0_108)</ins>

## Objective
To map an environment using a depth camera and then visualize the entire environment in 3D.

## Reasoning
There are multiple ways to map and visualize an environment in 2d that are available to users using ROS. The most common being using a lidar sensor and the Nav2 toolbox to map the environment and then RViz to view it. However, while there are ways to visualize the environment in 3D through the use of either 3D Lidar or Depth cameras, there are not too many easy solutions to visualize a completed 3d map due to how computationally heavy the data can be. Some of the available solutions involve using octomap (documentation can be found <ins>[here](http://octomap.github.io/)</ins>

Our project proposes utilizing the PyVista package available in python to map and visualize the 3D environment. You can read more about the package <ins>[here](https://docs.pyvista.org/)</ins>

## Background
The data being mapped is known as a point cloud. A point cloud is a set of discrete data points in space, each having their own x, y, z coordinates as well as other parameters such as the rgb color and the intensity. Point cloud data is usually stored in a .pcd file containing a header describing the data followed by the data

![image](https://github.com/jvinol/G1FinalProject/blob/16b4066c0564310ce5ae8aafed03647be812ef42/README%20Images/se-sample-museum.png)

**Figure:** Point cloud visualization of a museum room <ins>[Source](https://help.sketchup.com/en/scan-essentials-sketchup/sample-point-cloud-data)</ins>

# Files in this Directory
1. PCD Python Scripts: 

2. README Images: Contains the images used in the Readme.md file

3. src/basic robot: The directory containing all the files necessary to run the program: It contains the following directories:
    - Models: Contains the xacro files necesssary to simulate a robot in gazebo
    - World: Contains multiple .world files necessary to simulate the environment in gazebo. For this demonstration, we will be making use of the obstacles.world environment
    - basic_robot/read_points.py: The python script written in an attempt to parse the pointcloud2 data being transmitted through the /camera/points topic. (**Note:** We were unable to get this to work due to first a configuration error and then a RTPS_Transport_SHM Error which we were unable to solve)
    -  config: Contains the configuration files to intialize RViz so that the display setup is already setup to view the simulated ray data being transmitted due to the lidar, the map being generated due to the slam_toolbox and the pointcloud visualization due to the data being transmitted by the depth camera.
    -  launch: Contains the launch files to easily launch the nodes for Gazebo, robot_state_publisher and initialize robot in the gazebo environment.
    -  CMakeLists.txt: Contains a list of directions describing the project's source files and target.
    -  package.xml: This file defines properties about the package such as the package name, version numbers, authors, maintainers, and dependencies on other packages.

## Setup
 - This project utilized an Oracle Virtual Machine running Ubuntu 20.04 Focal Fossa and ROS2 Foxy Fitzroy.

 - To install Ubuntu 20.04 on a Virtual Machine, follow the instructions <ins>[here](https://linuxhint.com/install_ubuntu_virtualbox_2004/)</ins>
 
 - To install ROS2 Foxy, follow the instructions <ins>[here](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)</ins>

 - Install the colcon build tool

    <code>sudo apt install python3-colcon-common-extensions</code>

 - In addition this project utilizes the slam_toolbox to give the user an option to map their environment in 2D and the teleop_twist_keyboard to control the robot. To install the slam_toolbox, type the following into the command line:

    <code>sudo apt install ros-foxy-slam-toolbox</code>

 - To run the slam_toolbox, the following command can be used:

    <code>ros2 launch slam_toolbox online_async_launch.py</code>

 - To install teleop_twist keyboard, type the following in the command line:

    <code>sudo apt-get install ros-foxy-teleop-twist-keyboard</code>

 - To run the teleop_twist_keyboard, the following command can be used:

    <code>ros2 run teleop_twist_keyboard teleop_twist_keyboard</code>

 - In addition Gazebo is required to run the simulation. For this project Gazebo 11 was used. To install Gazebo, type the following in the command line:

    <code>curl -sSL http://get.gazebosim.org | sh</code>

    <code>sudo apt install ros-foxy-gazebo-ros-pkgs</code>

 - Ensure that git is installed by using the following command:

    <code>sudo apt install git</code>

 - Install a python package called xacro

    <code>sudo apt install python3-pip</code>

    <code>pip install xacro</code>

 - Lastly: 

    <code>sudo apt update && sudo apt upgrade</code>

## How to run the project

1. Change currect directory to the desktop
<code>cd ~/Desktop</code>

2. Clone this github repository on to the desktop. This will create a workspace G1FinalProject on the desktop. Change directories into the newly created workspace.

    <code>git clone https://github.com/jvinol/G1FinalProject</code>
    
    <code>cd ~/Desktop/G1FinalProject</code>

3. Source the ROS2 environment (This will need to be done everytime a new terminal is opened)
    
    <code>source /opt/ros/foxy/setup.bash</code>

    Alternatively, add the sourcing script to the .bashrc folder so that it runs everytime a new console is intialized. Remember to source the bashrc file in the current terminal once the sourcing script has been added.
    
    <code>echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc</code>
    
    <code> source ~/.bashrc</code>  
    
4. In the root of the workspace (Desktop/G1FinalProject) build the workspace using colcon
    
    <code>colcon build</code>
    
5. In a new terminal, source the workspace (do not forget to source the ROS2 environment first.
    <code>. install/setup.bash</code>
    
6. Run the launch program to launch gazebo, robot state publisher and spawn the robot. The world can be changed by changing the path specified in world:=./src/basic_robot/World/obstacles.world.

    <code>ros2 launch basic_robot launch_sim.py world:=./src/basic_robot/World/obstacles.world</code>

    We are currently using the obstacles.world, however other worlds are present in the World directory under the basic_robot directory in src directory. Example worlds can be found <ins>[here](https://github.com/chaolmu/gazebo_models_worlds_collection.git)</ins> and saved into the Worlds directory as well.
    
7. In a new command line terminal, run the teleop keyboard to control the robot in the gazebo environment. You can use the key inputs displayed to control the robot

    <code>ros2 run teleop_twist_keyboard teleop_twist_keyboard</code>
    
8. 


