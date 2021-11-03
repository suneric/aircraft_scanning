## TEST  

### Maps
- Map1: 30 meters x 30 meters, grid resolution is 1 meter, all grids are valid grid (green).
![](./image/map331.jpeg)

### Candidate viewpoints
- working distance: 3 meters
- fov: 70 degrees x 70 degrees
- resolution: 1 meters
- total count: 900
![](./image/vpcandidate-u-m1.jpeg)

### Minimum Set Covering Problem (try 1000 times and select the solution with the least number of viewpoints)
- 100% coverage: 49 viewpoints, 62% duplication (overlap grids), 26.67 unique duplication
![](./image/scp-m1-vp1-100c.jpeg)
- 98% coverage: 44 viewpoints, 42.67% duplication (overlap grids), 20.22% unique duplication
![](./image/scp-m1-vp1-98c.jpeg)
- 95% coverage: 40 viewpoints, 25.33% duplication (overlap grids), 12.44% unique duplication
![](./image/scp-m1-vp1-95c.jpeg)
- 92% coverage: 37 viewpoints, 15.33% duplication (overlap grids), 7.67% unique duplication
![](./image/scp-m1-vp1-92c.jpeg)
- 90% coverage: 36 viewpoints, 13.11% duplication (overlap grids), 6.56% unique duplication
![](./image/scp-m1-vp1-90c.jpeg)

### MAX-MIN Ant System on 92% coverage
- performance with different hyper-parameters ($\alpha \beta \rho$)
![](./image/MMAS-m1-rho005.png)
![](./image/MMAS-m1-rho02.png)
![](./image/MMAS-m1-rho05.png)
- the best configures (hyper-parameters)
![](./image/MMAS-m1-best.png)
- best tour
  - $\alpha=1, \beta=2, \rho=0.05$
![](./image/MMAS-m1-best-a1b2r005.jpeg)
  - $\alpha=5, \beta=7, \rho=0.05$
![](./image/MMAS-m1-best-a5b7r005.jpeg)
  - $\alpha=1, \beta=3, \rho=0.2$
![](./image/MMAS-m1-best-a1b3r02.jpeg)
  - $\alpha=1, \beta=5, \rho=0.2$
![](./image/MMAS-m1-best-a1b5r02.jpeg)
  - $\alpha=1, \beta=3, \rho=0.5$
![](./image/MMAS-m1-best-a1b3r05.jpeg)
  - $\alpha=1, \beta=5, \rho=0.5$
![](./image/MMAS-m1-best-a1b5r05.jpeg)


### Monte Carlo Tree Search
- performance with different hyper-parameters (nc: neighbor parameter, rc: reward parameter,  $\epsilon$: terminal epsilon)
![](./image/MCTS-m1-n4-e005.png)
![](./image/MCTS-m1-n4-e02.png)
- the best configures (hyper-parameters)
![](./image/MMAS-m1-n4-best.png)
- best tour
  - nc=0.5, rc=0.75, $\epsilon=0.2$
![](./image/MCTS-m1-best-n4-nc05rc075e02.jpeg)
  - nc=0.5, rc=0.75, $\epsilon=0.05$
![](./image/MMAS-m1-best-a5b7r005.jpeg)
