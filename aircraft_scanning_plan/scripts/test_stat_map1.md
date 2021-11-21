## TEST of Map1  

### Map
- 30 meters x 30 meters, grid resolution is 1 meter, all grids are valid grid (green).
- Candidate viewpoints
  - working distance: 3 meters
  - fov: 70 degrees x 70 degrees
  - resolution: 1 meters
  - total count: 900

map|candidate viewpoints
:----:|:----:
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/map331.jpeg">|<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/vpcandidate-u-m1.jpeg">

### Minimum Set Covering Problem (try 1000 times and select the solution with the least number of viewpoints)
- 100% coverage: 49 viewpoints, 62% duplication (overlap grids), 26.67 unique duplication
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/scp-m1-vp1-100c.jpeg" width=50% height=50%>
- 98% coverage: 44 viewpoints, 42.67% duplication (overlap grids), 20.22% unique duplication
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/scp-m1-vp1-98c.jpeg" width=50% height=50%>
- 95% coverage: 40 viewpoints, 25.33% duplication (overlap grids), 12.44% unique duplication
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/scp-m1-vp1-95c.jpeg" width=50% height=50%>
- 92% coverage: 37 viewpoints, 15.33% duplication (overlap grids), 7.67% unique duplication
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/scp-m1-vp1-92c.jpeg" width=50% height=50%>
- 90% coverage: 36 viewpoints, 13.11% duplication (overlap grids), 6.56% unique duplication
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/scp-m1-vp1-90c.jpeg" width=50% height=50%>

### MAX-MIN Ant System on 92% coverage
- performance with different hyper-parameters (alpha beta rho)

rho=0.05|rho=0.2|rho=0.5
:----:|:----:|:----:
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/MMAS-m1-rho005.png">|<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/MMAS-m1-rho02.png">|<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/MMAS-m1-rho05.png">

- the best configures (hyper-parameters)
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/MMAS-m1-best.png" width=50% height=50%>
- best tour
  - alpha=1, beta=3, rho=0.05
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/MMAS-m1-best-a1b2r005.jpeg" width=50% height=50%>
  - alpha=5, beta=7, rho=0.05
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/MMAS-m1-best-a5b7r005.jpeg" width=50% height=50%>
  - alpha=1, beta=3, rho=0.2
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/MMAS-m1-best-a1b3r02.jpeg" width=50% height=50%>
  - alpha=1, beta=5, rho=0.2
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/MMAS-m1-best-a1b5r02.jpeg" width=50% height=50%>
  - alpha=1, beta=3, rho=0.5
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/MMAS-m1-best-a1b3r05.jpeg" width=50% height=50%>
  - alpha=1, beta=5, rho=0.5
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/image/MMAS-m1-best-a1b5r05.jpeg" width=50% height=50%>


### Monte Carlo Tree Search
- neighbor viewpoints


### Comparison
- 30 meters X 30 meters, discritized with 900 square grid (length of 1 meter)
- number of candidate viewpoints: 900

- SCP+ACO:
- best solution: 26 viewpoints selected, travel distance is 138.13 meters.
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/acobest_1.png" width=50% height=50%>

- MCTS:
  - neighborhood viewpoints control parameter: 0.5
  - max simulation iteration: 100000
  - terminal epsilon: 0.1, with decay rate 0.9999

  - number of neighborhood viewpoints: 3
  - Number of iteration required to achieve 100% coverage
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/mcts_sigma.png" width=50% height=50%>
  - best solution: 25 viewpoints selected, travel distance is 168 meters, altered = 173.8 meters
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/results/mctsbest_1.png" width=50% height=50%>
