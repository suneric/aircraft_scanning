## TEST for aircraft scanning
- SCP+ACP
```python cpp_aco.py --load ../viewpoint/uav_scanning/ --vpsfile fuselage-d3r1-fov8080.txt --scIter 1000 --acIter 2000 --ants 100 --alpha 1 --beta 2 --rho 0.05
```
- MCTS
```python cpp_mcts.py --load ../viewpoint/uav_scanning/ --vpsfile fuselage-d3r1-fov8080.txt --cn 0.9 --ad 6 --tc 1 --cp 0.9 --dr 0.99999 --fe 0.1 --sn 1000000```


### Experiment Environment
- Target Aircraft
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/airline757.png" width=50% height=50%>
- UAV-Camera system
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/uav.png" width=50% height=50%>

- Octree representation of the aircraft
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/octree_representation.png" width=50% height=50%>

### upper part of fuselage
- working distance: 3 meters, 4 meters, and 5 meters
- resolution: 1 meter
- fov: 80 degrees x 80 degrees
- near clip: 0.1 meters
- far clip: 5.0 meters
- number of candidate viewpoints: 250
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/candidate_vps.png" width=50% height=50%>


### Comparison

- SCP+ACO (working distance = 3 meters), scIter 1000, ants 100, acIter 2000, alpha 1, beta 2, rho 0.05
  - 109 viewpoints selected, travel distance is 195.39 meters, computational cost: 903.79 s
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/acobest_f3.png" width=50% height=50%>

  - dense model: operation time: 908 s
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/densemodel_aco.png" width=50% height=50%>  

- MCTS (working distance = 3 meters), after alter the tour, computational cost: 4664.17 s
  - 55 viewpoints selected, travel distance 152.371 meters, 85.89% coverage, terminal e = 0.1, number of neighborhood viewpoints = 6, decay rate = 0.99999, simulation iteration count = 1,000,000, sigma = 0.9
  <img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/mctsbest_f3.png" width=50% height=50%>

  - dense model: operation time: 526 s
 <img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/densemodel_mcts.png" width=50% height=50%>  
