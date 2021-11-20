## TEST for aircraft scanning

### upper part of fuselage
- working distance: 3 meters, 4 meters, and 5 meters
- resolution: 1 meter
- fov: 80 degrees x 80 degrees
- near clip: 0.1 meters
- far clip: 5.0 meters
- number of candidate viewpoints: 250
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/aircraft/vpsfuselage-wd3r1.jpg" width=50% height=50%>


### Comparison

- SCP+ACO (working distance = 3 meters)
  - 109 viewpoints selected, travel distance is 195.39 meters
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/aircraft/acobest_f3.jpg" width=50% height=50%>

- SCP+ACO (working distance = 4 meters)
  - 78 viewpoints selected, travel distance is 189.06 meters
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/aircraft/acobest_f4.jpg" width=50% height=50%>

- SCP+ACO (working distance = 5 meters)
  - 56 viewpoints selected, travel distance is 163.66 meters
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/aircraft/acobest_f5.jpg" width=50% height=50%>

- MCTS (working distance = 3 meters)
  - 54 viewpoints seleted, travel distance 292.297 meters, 84.04% coverage, terminal $\epsilon$ = 0.01
  <img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/aircraft/mctsbest_fuselage_f3.jpg" width=50% height=50%>

- MCTS (working distance = 4 meters)
  - 46 viewpoints seleted, travel distance 325.544 meters, 85.01% coverage, terminal $\epsilon$ = 0.1
<img src="https://github.com/suneric/aircraft_scanning/blob/master/aircraft_scanning_plan/scripts/aircraft/mctsbest_fuselage_f4.jpg" width=50% height=50%>
