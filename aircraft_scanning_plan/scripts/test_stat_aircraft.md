## TEST for aircraft scanning

### Experiment Environment
- Target Aircraft
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/uav.png" width=50% height=50%>
- UAV-Camera system
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/airline757.png" width=50% height=50%>

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

- SCP+ACO (working distance = 3 meters)
  - 109 viewpoints selected, travel distance is 195.39 meters
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/acobest_f3.png" width=50% height=50%>

  - dense model
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/densemodel_aco.png" width=50% height=50%>  

- MCTS (working distance = 3 meters), after alter the tour
  - 55 viewpoints selected, travel distance 152.371 meters, 85.89% coverage, terminal e = 0.1, number of neighborhood viewpoints = 6, decay rate = 0.99999, simulation iteration count = 1,000,000, sigma = 0.9
  <img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/aircraft/mctsbest_f3.png" width=50% height=50%>

  - dense model
 <img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/densemodel_mcts.png" width=50% height=50%>  
