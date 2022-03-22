# aircraft scanning
This is a project of using a quadrotor equiped with a realsense camera to acquire the 3D point cloud of an aircraft.
The project uses
-[ROS hector_gazebo_plugin](http://wiki.ros.org/hector_gazebo_plugins) for quadrotor modeling and control,
-[Point Cloud Library](http://pointclouds.org/) for 3D point cloud data processing and visualization.
-[Gazebo9](http://gazebosim.org/) for dynamic simulation
The system is developed in python and C++ on Ubuntu 16.04 with [ROS Kinetic](http://wiki.ros.org/kinetic).

## Dependencies
1. install point cloud library (pcl 1.7.2 on ubuntu 16.04, pcl 1.8.1 on ubuntu 18.04) and [python-pcl](https://python-pcl-fork.readthedocs.io/en/rc_patches4/install.html)(use prebuild 0.3 for pcl1.8)
```
sudo apt-get install libpcl-dev -y
```
2. install [opencv](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html) and opencv-python (4.2.0.32)
```
sudo apt-get install libopencv-dev
pip install opencv-python==4.2.0.32
```
3. install [Gazebo9](http://gazebosim.org/tutorials?tut=install_ubuntu) with ROS Kinetic or Melodic
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get install gazebo9
```
4. install [realsense sdk](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) and [realsense-ros](https://github.com/IntelRealSense/realsense-ros)(compile from source)
```
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev (for compile realsense-ros)

```
some dependencies for gazebo-ros-pkgs are required.
  - ros-kinetic-gazebo-ros-pkgs
  - ros-kinetic-gazebo-ros-control
  - ros-kinetic-effort-controllers
  - ros-kinetic-joint-state-controller
  - ros-kinetic-controller-manager
  - ros-kinetic-geographic-msgs   
  - ros-kinetic-sensor-msgs

5. install hector_quadrotor and related packages
  - [hector_quadrotor](http://wiki.ros.org/hector_quadrotor)
  - [hector_gazebo](http://wiki.ros.org/hector_gazebo)
  - [hector_localization](http://wiki.ros.org/hector_localization)
  - [hector_sensor_description](http://wiki.ros.org/hector_sensors_description)
6. install [iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack)

## packages
- aircraft_scanning_description. the package of modeling the quadrotor and the realsense camera.
- aircraft_scanning_control. the package of controlling the quadrotor and the realsense camera.
- aircraft_scanning_visualize. the package of visualizing the point cloud acqruired by realsense camera.
- aircraft_scanning_gazebo. the package of gazebo simulation
- aircraft_scanning_teleop. the package of teleop operation for controlling the quadrotor and realsense camera.
- aircraft_scanning_plan. the package of path planning with MonteCalorTreeSearch and MAX-MIN Ant System (MMAS)

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

## point cloud process
- merge multiple pcd file into one file: asv3d -m folder filter[0|1] resolution[0.05 meters]
```
cd catkin_ws
devel/lib/aircraft_scanning_visualize/asv3d -m ~/Temp/Scanning/ 1 0.05
```

- visualize point cloud and trajectory: asv3d -v folder resolution[1.0 meter] display[-1|-2|-3|-4|other] trajectory[optional]
```
cd catkin_ws
devel/lib/aircraft_scanning_visualize/asv3d -v ~/Temp/Data/ 1.0 -1 trajectory.txt  
```

- generate viewpoints: asv3d -t folder distance[3.0 meters] resolution[1.0 meter] display[-1|-2|-3|-4|other] config[optional]
```
cd catkin_ws
devel/lib/aircraft_scanning_visualize/asv3d -t ~/Temp/Data/ 3.0 1.0 -1 ~/catkin_ws/src/aircraft_scanning/aircraft_scanning_visualize/config/fuselage.txt
```
## trajectory generation
```
cd catkin_ws/src/aircraft_scanning/aircraft_scanning_plan/scripts/
```

- SCP+ACP

```
python cpp_aco.py --load folder --vpsfile vps.txt --trajfile traj.txt --scIter 1000 --acIter 2000 --ants 100 --alpha 1 --beta 2 --rho 0.05
```
- MCTS

```
python cpp_mcts.py --load folder --vpsfile vps.txt --trajfile traj.txt --cn 0.9 --ad 6 --tc 1 --cp 0.9 --dr 0.99999 --fe 0.1 --sn 1000000
```

- Alter trajectory

```
python test.py --loadvps folder --vpsfile vps.txt --savebest folder --alter True
```

- Mirror trajectory (y-zplane)
```
python test.py --loadvps folder --vpsfile vps.txt --savebest folder --mirror True
```


## Developer
Yufeng Sun | sunyf@mail.uc.edu | IRAS Lab @ University of Cincinnati


## Reference
- https://rosindustrial.org/3d-camera-survey
