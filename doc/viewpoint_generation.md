# Viewpoint Generation

## camere position generated based on point cloud data
A octree representation of the rough point cloud will be used for generating viewpoints (a list of camera pose), with specified resolution and safe distance. The safe distance is the minimum distance from camera to the aircraft surface, while a specified height will be considered for generating the viewpoints in the height range because of the limited workspace of robot.

## command - generate viewpoints
```
/path/to/asv3d -t [/path/to/model] [safe distance] [octree resolution] [display type] [configuration]
```
  - [/path/to/model]: the path of aircraft model
  - [safe distance]: for example "0.5" meters
  - [octree resolution]: for example "1.0" meters
  - [display type]: -1 for camera coordinate
  - [configuration]: aircraft segmentation bounding box and limited height range
The "viewpoints.txt" will be generated and saved to "~/Temp/" folder

## trajectory optimization based on coverage
To find a sub optimal trajectory which consists of as less viewpoints as possible and largest coverage of the aircraft surface. The frustum culling will be used to tell the voxels covered by each viewpoint, thus a total voxels coverage can be calculated and referenced by a Monte Carlo Tree Search (MCTS) algorithm.

## command - learning optimal trajectory
```
cd ~/aircraft_scanning/aircraft_scanning_plan/scripts
python mcts_train.py --sn=[number of iteration] --ad=[action dimension] -- tc=[terminal coverage] --cp=[control parameter] --dr=[decay rate of epsilon] --fe=[final epsilon]
```
  - [number of iteration]: number of iteration want to take, example: 100000
  - [action dimension]: number of neighbor viewpoint for next visit, example: 8
  - [terminal coverage]: desired terminal coverage, example: 0.95 for 95% coverage
  - [control parameter]: control parameter for best action evaluation, example: 0.3, should be in [0,1]
  - [decay rate of epsilon]: decay rate of epsilon, example : 0.999
  - [final epsilon]: terminal epsilon, large epsilon result long time learning but more optimal trajectory, example: 0.2, should be in [0,1]
