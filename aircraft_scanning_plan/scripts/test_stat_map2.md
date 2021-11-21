## TEST of Map2  

### Map
- 90 meters x 60 meters, grid resolution is 1 meter, 4760 grids are valid grid (green) in total 5400 grids .
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/map961.jpeg" width=50% height=50%>

- Candidate viewpoints
  - working distance: 5 meters
  - fov: 80 degrees x 80 degrees
  - resolution: 3 meters
  - total count: 600

### Comparison

- SCP+ACO
  - 91 viewpoints selected, travel distance is 640.16 meters
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/acobest_2.png" width=50% height=50%>  

- MCTS
 - 68 viewpoints selected, altered travel distance is 576.93 meters, coverage is 82.04%, terminated e = 0.1, simulation iteration count 100,000   
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/mctsbest_2.png" width=50% height=50%>   
