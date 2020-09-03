# mobile base
- wheel radius: 0.15 meters
- wheel width: 0.1 meters
- chassis length: 1 meter
- chassis width: 0.8 meters
- chassis height: 0.25 meters
- support base height: 0.5 meters
- footprint center: (0, 0, 0.15)
- support top to foorprint center: 0.7 meter   

- drive type: skid steering (differential drive)

# manipulator (KUKA iiwa 14)
- link 0-2: 0.36 meters (0.34 meters for iiwa 7)
- link 2-4: 0.42 meters (0.4 meters for iiwa 7)
- link 4-6: 0.4 meters (0.4 meters for iiwa)
- link 6-E: 0.126 meters

# camera (realsense d435)
- width: 0.09 meters
- height: 0.025 meters
- depth: 0.025 meters
- front to depth: 0.0042 meters
- z: 0.0208 meters (depth - front to depth)

# quadrotor:
- camera link 1 length: 0.42 (from quadrotor center to camera joint)
- camera link 2 length: 0.0358 (from camera joint to front of camera)
