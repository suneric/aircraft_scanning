# aircraft scanning
This is a project of using a quadrotor equiped with a realsense camera to acquire the 3D point cloud of an aircraft.
The project uses [ROS hector_gazebo_plugin](http://wiki.ros.org/hector_gazebo_plugins) for quadrotor modeling and control, [Point Cloud Library](http://pointclouds.org/) for 3D point cloud data processing and visualization. The system is developed in python and C++ on Ubuntu 16.04 with [ROS Kinetic](http://wiki.ros.org/kinetic).

## packages
- aircraft_scanning_description. the package of modeling the quadrotor and the realsense camera.
- aircraft_scanning_control. the package of controlling the quadrotor and the realsense camera.
- aircraft_scanning_visualize. the package of visualizing the point cloud acqruired by realsense camera.
- aircraft_scanning_gazebo. the package of gazebo simulation
- aircraft_scanning_teleop. the package of teleop operation for controlling the quadrotor and realsense camera.

## features
- start gazebo simulation with teleop

```
roslaunch aircraft_scanning_gazebo uav_scanning_teleop.launch
```

- start gazebo simulation with autonomous fly

```
roslaunch aircraft_scanning_gazebo uav_scanning_auto.launch
```

- visualize the point cloud

```
roslaunch aircraft_scanning_visualize point_cloud_visualize.launch
```

## Developer
Yufeng Sun | sunyf@mail.uc.edu | IRAS Lab @ University of Cincinnati
