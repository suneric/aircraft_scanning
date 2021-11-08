## TEST of Map2  

### Map
- 90 meters x 60 meters, grid resolution is 1 meter, 4760 grids are valid grid (green) in total 5400 grids .
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/map961.jpeg" width=50% height=50%>

- Candidate viewpoints
  - working distance: 3 meters
  - fov: 70 degrees x 70 degrees
  - resolution: 1 meters
  - total count: 5400

### Set covering problem solver
  - find 213 viewpoints from 5400 viewpoints candidate

### MAX-MIN Ant System on 92% coverage
  - performance with different hyper-parameters (alpha = 1 beta = 3 rho = 0.05)
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/MMAS-m2-best-a1b3r005.jpeg" width=50% height=50%>

### Monte Carlo Tree Search
