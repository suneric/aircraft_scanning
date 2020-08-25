# aircraft scanning
This is a project of using a quadrotor equiped with a realsense camera to acquire the 3D point cloud of an aircraft.
The project uses
-[ROS hector_gazebo_plugin](http://wiki.ros.org/hector_gazebo_plugins) for quadrotor modeling and control,
-[Point Cloud Library](http://pointclouds.org/) for 3D point cloud data processing and visualization.
-[Gazebo9](http://gazebosim.org/) for dynamic simulation
The system is developed in python and C++ on Ubuntu 16.04 with [ROS Kinetic](http://wiki.ros.org/kinetic).

## Dependencies
1. install [Gazebo9](http://gazebosim.org/tutorials?tut=install_ubuntu) with ROS Kinetic
(by default, install ROS kinetic will install Gazebo7, so install ros-kinetic-desktop instead of ros-kinetic-desktop-full)
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get install gazebo9
```
some dependencies for gazebo-ros-pkgs are required.
  - ros-kinetic-gazebo-ros-pkgs
  - ros-kinetic-gazebo-ros-control
  - ros-kinetic-effort-controllers
  - ros-kinetic-joint-state-controller
  - ros-kinetic-controller-manager
  - ros-kinetic-geographic-msgs   
  - ros-kinetic-sensor-msgs
2. install point cloud library (by default pcl 1.7.2 on ubuntu 16.04) and python-pcl
3. install [opencv](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html) and opencv-python
4. install hector_quadrotor and related packages
  - [hector_quadrotor](http://wiki.ros.org/hector_quadrotor)
  - [hector_gazebo](http://wiki.ros.org/hector_gazebo)
  - [hector_localization](http://wiki.ros.org/hector_localization)
  - [hector_sensor_description](http://wiki.ros.org/hector_sensors_description)
5. install [iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack)

## packages
- aircraft_scanning_description. the package of modeling the quadrotor and the realsense camera.
- aircraft_scanning_control. the package of controlling the quadrotor and the realsense camera.
- aircraft_scanning_visualize. the package of visualizing the point cloud acqruired by realsense camera.
- aircraft_scanning_gazebo. the package of gazebo simulation
- aircraft_scanning_teleop. the package of teleop operation for controlling the quadrotor and realsense camera.
- aircraft_scanning_plan. the package of path planning with MonteCalorTreeSearch

## features
- start gazebo simulation with teleop (make sure the teleop_twist_joy is installed)
```
roslaunch aircraft_scanning_gazebo uav_scanning_teleop.launch
```
- start gazebo simulation with autonomous fly
```
roslaunch aircraft_scanning_gazebo uav_scanning_auto.launch
```
- visualize the point cloud
```
cd ~/catkin_ws
/devel/lib/aircraft_scanning_visualize/asv3d [task-type] [...]
```
- scanning path training with MCTS
```
cd ~/catkin_ws/src/aircraft_scanning/aircraft_scanning_plan/scripts/
python mcts_train.py --sn <simulation count> --ad <neighbors count> --tc <terminal coverge> --cp <control param> --dr <decay ratio> --fe <final epsilon>
```

## Developer
Yufeng Sun | sunyf@mail.uc.edu | IRAS Lab @ University of Cincinnati
