# ROS Navigation Interface
A ROS navigation app that utilizes a SLAM generated map to navigation a house enviroment in Gazebo using a low-level path planner. The app allows you to navigate to 6 different location in the house, the location are seen here.  
# Prerequisite Installation
## Installing ROS (Melodic)
```console
sudo apt update
sudo apt upgrade
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh
chmod 755 ./install_ros_melodic.sh 
bash ./install_ros_melodic.sh
```
## Installing ROS Dependencies 
```console
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
  ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
  ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
  ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
  ros-melodic-rosserial-server ros-melodic-rosserial-client \
  ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
  ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
  ros-melodic-compressed-image-transport ros-melodic-rqt* \
  ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
  ```
## Installing TurtleBot3 Package
```console
sudo apt-get install ros-melodic-dynamixel-sdk
sudo apt-get install ros-melodic-turtlebot3-msgs
sudo apt-get install ros-melodic-turtlebot3
```
## Setting up Bash enviroment
```console
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
echo "source ~/roboticsfinalproject/devel/setup.bash"
source ~/.bashrc
```
# Running the Simulator

```console 
roslaunch turtlebot3_gazebo turtlebot3_house.launch &
roslaunch navigation_interface navigation.launch
cd ~/roboticsfinalproject/
rosrun navigation_interface navigation_interface
```
# User Interface
Keyboard command to location mapping
- 1 = Room 1 
- 2 = Room 2
- 3 = Room 3
- 4 = Room 4
- 5 = Room 5
- 6 = Room 6
