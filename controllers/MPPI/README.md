
# MPPI Controller Simulation

## Overview
This Python script simulates a Model Predictive Path Integral (MPPI) controller for trajectory following in a 2D environment with dynamic obstacles. The robot is modeled as a point mass with velocity and steering dynamics, controlled by the MPPI algorithm to follow a predefined sinusoidal trajectory. Randomly placed obstacles are scattered around the trajectory, and the controller locally adjusts the path to avoid them.

## Key Features
- Implements the MPPI algorithm as a local planner and controller.
- Simulates a point-mass robot with velocity and steering constraints.
- Generates a sinusoidal reference trajectory and randomly places obstacles.
- Visualizes rollouts, the optimal trajectory, and the robot's path in real-time using Matplotlib.

## Important Notes
**This is a local planner** (see diploma thesis for details). As such, it focuses on short-term trajectory tracking and obstacle avoidance. In complex environments with intricate obstacle layouts, it may lead to "dead ends" or suboptimal paths. This is expected behavior, as MPPI is designed to work in tandem with a global planner (e.g., in a navigation stack) that resolves such situations by replanning the trajectory. The MPPI controller here tracks the trajectory while accounting for dynamic obstacles not anticipated in the global plan.

This is a standalone simulation, not a ROS 2 node, but it clearly demonstrates the MPPI algorithm's implementation, which can be adapted into a ROS 2 node for real-world use.

## Dependencies
- Python 3
- `numpy`
- `matplotlib`

## Installation
1. Ensure dependencies are installed:
   ```bash
   pip install numpy matplotlib
   ```
2. Save the script as `mppi_simulation.py` in your working directory.

## Usage
Run the simulation directly:
```bash
python mppi_simulation.py
```

### What to Expect
- A sinusoidal trajectory (green dashed line) is generated from (0, 3) to (10, ~2).
- 30 circular obstacles (black) are randomly placed near the trajectory.
- The robot starts at the blue dot and aims for the red dot (goal).
- Rollouts (colored lines) show sampled trajectories, with the optimal path in dark gray.
- The robot’s current position is marked with a red "+".
- The simulation runs for 800 steps, updating in real-time.

## Configuration
- **Trajectory**: Edit `generate_trajectory()` to change the reference path.
- **Obstacles**: Adjust `n_obstacles` and the random range in the script.
- **MPPI Parameters**: Modify `horizon`, `n_samples`, `lambda_`, `nu`, `R`, `cov`, and `dt` to tune the controller.

## Limitations
- The robot is a point mass without physical dimensions.
- No collision detection beyond cost penalties.
- Obstacles are static but randomly placed each run.

## License
This simulation is released under the Apache-2.0 License.


