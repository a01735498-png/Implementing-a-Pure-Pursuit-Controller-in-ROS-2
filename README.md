# Pure Pursuit Controller — prius_hybrid (Gazebo + ROS 2)

Pure Pursuit controller for the `prius_hybrid` vehicle in Gazebo simulation, built on ROS 2 Jazzy. The car records a reference trajectory via teleoperation, then follows it autonomously, and the tracking performance is evaluated afterward.

## Repository structure

```
.
├── src/
│   ├── prius_bringup/                # Simulation package: vehicle model and Gazebo world
│   └── pure_pursuit_controller/      # ROS 2 package (ament_python) with the controller
│       ├── pure_pursuit_controller/
│       │   ├── path_recorder.py      # Records /odom to a waypoints CSV
│       │   └── pure_pursuit_node.py  # Pure Pursuit control node
│       ├── launch/
│       │   └── pure_pursuit.launch.py  # Starts simulation + controller together
│       └── config/
│           └── waypoints.csv         # Default reference trajectory
└── evaluate_tracking_charts_cvs/     # Performance analysis (ROS-independent)
    ├── evaluate_tracking.py          # Computes cross-track error and generates plots
    ├── waypoints.csv                 # Reference used in the evaluated run
    ├── actual_trajectory.csv         # Trajectory actually executed by the vehicle
    ├── tracking_report.png           # Reference vs. executed trajectory (overlay)
    ├── tracking_report_errors.csv    # Per-point cross-track error
    └── diagnostico_error.png         # Error and curvature vs. distance traveled
```

## Requirements

- ROS 2 Jazzy
- Gazebo (gz sim)
- Python 3.12, with `pandas`, `numpy`, `matplotlib` for the analysis script:
```bash
  pip install pandas numpy matplotlib --break-system-packages
```

## Vehicle parameters

Taken directly from `src/prius_bringup/models/prius_hybrid/model.sdf` (`AckermannSteering` plugin):

| Parameter | Value |
|---|---|
| Wheelbase (`L`) | 2.7 m |
| Steering limit (`steering_limit`) | 0.5 rad (~28.6°) |
| Maximum curvature (`κ_max = tan(steering_limit)/L`) | ≈0.202 1/m |

## Usage

### 1. Build the workspace

```bash
cd ~/Workspaces/pure_pursuit_ros2
colcon build --packages-select pure_pursuit_controller
source install/setup.bash
```

### 2. Record a reference trajectory

```bash
ros2 launch prius_bringup gz_sim.launch.py
```
In another terminal:
```bash
ros2 run pure_pursuit_controller path_recorder --ros-args \
  -p output_file:=/home/user_name/Workspaces/pure_pursuit_ros2/waypoints.csv
```
Drive the car via teleoperation along the desired path, then stop recording with `Ctrl+C`.

### 3. Autonomous tracking

```bash
ros2 launch pure_pursuit_controller pure_pursuit.launch.py \
  waypoints_file:=/home/user_name/Workspaces/pure_pursuit_ros2/waypoints.csv
```
This launches the simulation and starts the controller automatically after a short delay, so the vehicle is settled at its spawn point before tracking begins.

### 4. Evaluate performance

To compare the reference against what the vehicle actually did, a second trajectory is recorded with `path_recorder` while the controller drives autonomously (see steps above), and then:

```bash
cd evaluate_tracking_charts_cvs/
python3 evaluate_tracking.py waypoints.csv actual_trajectory.csv --out tracking_report
```

This prints the cross-track error (mean, RMS, max) and generates `tracking_report.png` with both trajectories overlaid.


## Known notes

- The controller uses an adaptive lookahead `P = k·v` with a minimum floor.
- `/odom` can deliver out-of-order messages; the node discards any message whose timestamp is earlier than the last one received.
