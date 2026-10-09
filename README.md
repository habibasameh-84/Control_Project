## Student information:
Name: Habiba Sameh Tawfik

## System architecture: 
1. Path Initialization: The Track Environment node generates and broadcasts the global reference trajectory on the `/path` topic, containing waypoints and target headings.
2. Feedback State Estimation: The `bicycle_sim_node` publishes the current 4D vehicle pose and dynamics on the `/state` topic at each dynamic time step dt.
3. Closed-Loop Control Execution:
  -The active Controller Node (`pid_node`, `pure_pursuit_node`, or `mpc_node`) ingests the current `/state` and `/path`.
  -It calculates longitudinal and lateral tracking errors and computes the necessary control actions.
4. Physics & State Forward Integration:
  -The `bicycle_sim_node` receives `/throttle` and `/steer`, applies powertrain acceleration gain, drag, and rolling resistance forces, and integrates the Kinematic Bicycle equations forward in time.
  -Updated dynamic states are broadcast back to the ROS 2 system via `/state`, `/joint_states`, and dynamic TF transforms (`map -> base_link`).
5. Decoupled Performance Observation:
  -The `lap_analyzer` node passively listens to `/state` and `/path`.
  -It computes orthogonal cross-track errors, heading misalignment, lap times, and RMS metrics without affecting controller execution.
  -It streams real-time scalar telemetry to PlotJuggler and visual markers/HUD overlays to RViz2.

## Mathematical formulations:

### Lateral PID :
The steering angle is calculated using a combined PID control law on Cross-Track Error (CTE) with Heading Error correction:
1. Heading Error Normalization:
  normalized_heading_error = atan2(sin(heading_err), cos(heading_err))
2. Integral CTE (with Anti-Windup Clamping):
  integral_cte = clamp(integral_cte + cte * dt, -integral_limit, integral_limit)
3. Derivative CTE:
  derivative_cte = (cte - prev_cte) / dt
4. Raw Steering Command:
  raw_steer = - ( Kp * cte + Ki * integral_cte + Kd * derivative_cte + k_yaw * normalized_heading_error )
5. Output Steering Limit (Clamping):
  steering_angle = clamp(raw_steer, -max_steer_rad, max_steer_rad)

### Longitudinal PID :
The Longitudinal PID controller regulates vehicle speed by translating the velocity tracking error into a normalized control effort u in [-1.0, 1.0] (mapped to throttle and braking effort) to compensate for drag and friction:
1. Velocity Error Calculation:
  e_v(t) = target_vel - current_vel
2. Proportional Term:
  P_term = Kp * e_v(t)
3. Integral Term with Anti-Windup Clamping:
  Integral_v(t) = clamp( Integral_v(t - dt) + e_v(t) * dt, -integral_limit, integral_limit )
  I_term = Ki * Integral_v(t)
4. Derivative Term:
  D_term = Kd * ( (e_v(t) - e_v(t - dt)) / dt )
5. Total Raw Control Effort & Output Clamping:
  u_raw = P_term + I_term + D_term
  u_clamped = clamp( u_raw, -max_brake, max_throttle )

### Pure Pursuit:
The steering angle is calculated using adaptive lookahead distance, target point local transformation, and geometric arc geometry:
1. Adaptive Lookahead Horizon:
  lookahead = clamp(kv * |v| + l_min, l_min, l_max)
2. Local Frame Transformation:
   y_local = -dx * sin(yaw) + dy * cos(yaw)
  (where dx = target_x - x, dy = target_y - y)
3. Pure Pursuit Arc Law (Steering Angle):
  alpha_steer = atan2(2 * L * y_local, lookahead^2)
4. Output Steering Limit (Clamping):
  steering_angle = clamp(alpha_steer, -max_steer_rad, max_steer_rad)

### MPC :
The MPC controller formulates optimal control as a constrained non-linear optimization problem solved over a horizon N using SciPy (SLSQP solver):
1. State Forward-Kinematics (Discrete Extended Bicycle Model):
  x[k+1]   = x[k] + v[k] * cos(yaw[k]) * dt
  y[k+1]   = y[k] + v[k] * sin(yaw[k]) * dt
  yaw[k+1] = yaw[k] + (v[k] / L) * tan(delta_k) * dt
  v[k+1]   = v[k] + a_k * dt
2. Tracking Errors (Path-Aligned Frame):
  cte         = -dx * sin(yaw_ref) + dy * cos(yaw_ref)
  heading_err = atan2(sin(yaw - yaw_ref), cos(yaw - yaw_ref))
  v_err       = v - v_ref
3. Quadratic Objective Cost Function:
  Cost = Sum over k ( w_lat * cte^2 + w_yaw * heading_err^2 + w_v * v_err^2 + w_steer * delta_k^2 + w_dsteer * (delta_k - prev_delta)^2 + w_accel * a_k^2 )
4. Variable Bounds & Control Mapping:
  Steering limit: -max_steer_rad <= delta_k <= max_steer_rad
  Acceleration limit: -k_a <= a_k <= k_a
  Normalized Throttle Command: throttle_cmd = clamp(a_0 / k_a, -1.0, 1.0)

### Kinematic Bicycle Model Simulator:
The vehicle motion and physics are updated over time step dt using the Kinematic Bicycle Model:
1. State Derivatives:
  x_dot     = v * cos(theta)
  y_dot     = v * sin(theta)
  theta_dot = (v / L) * tan(delta)
  v_dot     = (k_a * u_throttle) - (c_drag * v^2) - (c_roll * v)
2. Forward Integration & Bounds:
  x     = x + x_dot * dt
  y     = y + y_dot * dt
  theta = atan2(sin(theta + theta_dot * dt), cos(theta + theta_dot * dt))
  v     = clamp(v + v_dot * dt, 0.0, max_speed)

### Lap Analyzer:
1. Segment Vector & Projection Parameter (t):
  dx = x2 - x1
  dy = y2 - y1
  t = clamp( ((x - x1) * dx + (y - y1) * dy) / (dx^2 + dy^2), 0.0, 1.0 )
2. Orthogonal Projected Point (px, py):
  px = x1 + t * dx
  py = y1 + t * dy
3. Signed Cross-Track Error (CTE):
  distance = sqrt( (x - px)^2 + (y - py)^2 )
  cross_product = dx * (y - y1) - dy * (x - x1)
  signed_cte = copysign(distance, cross_product)

## Benchmark table over at least three full laps:
| Controller Mode | Best Lap Time (s) | Top Speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Laps Completed / Status |
|---|---|---|---|---|---|---|
| **Manual Teleoperation** (very hard to implement)
| **Lateral PID (Reactive)** | 81.33 s | 7.49 m/s | 0.381 m | 2.113 m | RMS = 0.520 m | 3 laps completed |
| **Pure Pursuit (Preview)** | 72.10 s | 7.26 m/s | 0.034 m | 0.360 m | 0.056 m | 3 laps completed |
| **Extended Kinematic MPC (Optimal)** | 114.30 s | 4.14 m/s | 0.065 m | 0.367 m | 0.088 m | 5 laps completed |

## Critical comparison of the controllers:
1. Lateral PID:
  Pros: lowest computational cost and simplest implementation.
  Cons: High tracking error and high instability at sharp corners due to lack of path foresight and integral windup.

2. Pure Pursuit:
  Pros: Very good balance between efficiency and trajectory tracking, achieving the best overall lap time.
  Cons: Sensitive to lookahead tuning; requires heuristic scaling based on speed.

3. Extended Kinematic MPC:
  Pros: Highest consistency and stability over long runs, maintaining smooth steering and minimal deviations without overshoot.
  Cons: Highest computational overhead slowes down simulation rate, leading to conservative top speed and higher lap times.

## Theoretical explanation of why MPC tracks better than Pure Pursuit and Lateral PID:
-The PID controller is purely reactive it only responds after an error e(t) has already occurred. In contrast, MPC evaluates a predictive horizon using the dynamic model of the vehicle, anticipating sharp turns before reaching them, therefore PID was faster than MPC but less accurate.
-While Pure Pursuit uses geometric arc heuristics to target a single point ahead, MPC solves a constrained optimization problem. It optimizes speed and steering simultaneously while considering constraints as physical vehicle boundaries and acceleration limits, preventing divergence across multiple laps.

## Milestone 6 summary:
**2D vs 3D Simulation**: The computational and modeling trade-offs between lightweight 2D kinematic simulation and full 3D physics engines (Gazebo / MVSim).
1. Kinematic Simulation vs. 3D Physics Simulation
Simulation is important for developing autonomous racing cars because it allows control algorithms and vehicle behavior to be tested before deployment on a physical vehicle.
A 2D kinematic bicycle model represents the vehicle’s motion using simplified equations for position, heading, velocity, and steering angle. It assumes idealized rolling conditions and does not explicitly model tire forces, suspension, or tire slip and because of its simplicity, it requires relatively little computational power and is suitable for developing and tuning trajectory-tracking controllers such as PID, Pure Pursuit, and Model Predictive Control (MPC) while a 3D physics simulator such as Gazebo can represent the vehicle’s physical structure, wheel-ground contact, and interactions with the environment. Depending on the physics engine and vehicle model, it can simulate effects such as friction, tire slip, and suspension behavior. It can also provide simulated sensor data for testing perception algorithms and their integration with vehicle control.
The Ackermann Vehicle in Modern Gazebo demonstrates how an Ackermann-steered vehicle can be integrated with ROS 2, sensors, and autonomous navigation components. Similarly, the Classic Gazebo Ackermann Simulation provides a vehicle model with steering and velocity control and a front camera for vision-based tasks.

2. Computational Cost and Modeling Trade-offs
The main advantage of a 2D kinematic simulation is computational efficiency. Its simplified equations allow many experiments to be performed quickly, making it useful for testing different controller parameters and comparing trajectory-tracking performance. In an autonomous racing project, performance can be evaluated using metrics such as lap time, cross-track error, and deviation from the reference trajectory. However, a kinematic model cannot fully reproduce the behavior of a racing car near its handling limits. At high speeds or during sharp turns, tire forces and slip can significantly affect the vehicle’s motion. Therefore, a controller that performs well in a simplified simulation may not achieve the same performance when realistic vehicle dynamics are considered.
Gazebo provides a more detailed simulation environment but generally requires more computational resources and careful configuration of the vehicle model, physical parameters, and sensor settings. Its results also depend on the accuracy of these configurations; a 3D model is not automatically a perfectly realistic model.
MVSim provides another option. MVSim supports 2D rigid-body physics, vehicle simulation, sensor simulation, and 2D/3D visualization. It can therefore be useful for efficient vehicle experiments and ROS 2 integration without requiring the full complexity of a high-fidelity 3D physics environment.

3. Applications in Autonomous Racing
For an autonomous racing team, the choice of simulator depends on the development stage and the system being tested.
* 2D kinematic simulation: Suitable for initial controller development, trajectory tracking, parameter tuning, and comparing algorithms under repeatable conditions.
* Gazebo: Suitable for testing vehicle models, wheel-ground interactions, simulated sensors, and the integration of perception and control components.
* MVSim: Suitable for lightweight vehicle simulation, ROS 2 integration, sensor experiments, and vehicle-related testing with relatively low computational overhead.
For example, a racing team can initially use a kinematic bicycle model to tune a Pure Pursuit or MPC controller and evaluate how accurately the vehicle follows the racing line. The controller can then be tested in a more detailed Gazebo simulation to investigate how tire grip, vehicle dynamics, and sensor measurements affect its performance.
Practical approaches to integrating vehicle steering, velocity commands, sensor data, and ROS 2 communication are particularly relevant when developing perception-based racing systems, where camera or LiDAR data must be processed to identify track boundaries and obstacles before the vehicle’s controller determines its motion.

# Conclusion:
2D kinematic simulation and 3D physics simulation serve different but complementary purposes in autonomous racing. A kinematic model is computationally efficient and suitable for rapid controller development, while Gazebo provides a more detailed environment for investigating vehicle dynamics and testing sensor integration. MVSim offers a lightweight alternative for vehicle simulation and ROS 2 experimentation.

# Simulation Environments Comparison
| Feature / Metric | 2D Kinematic Model | MVSim Simulator | 3D Physics (Gazebo) |
| --- | --- | --- | --- |
| **Computational Cost** | Extremely Low (Real-time Fast Execution) | Low to Moderate | High (GPU/CPU Intensive) |
| **Primary Use Case** | Controller Tuning (PID, Pure Pursuit, MPC) & Benchmark Analysis | Lightweight Vehicle Simulation & Basic ROS 2 Integration | Full Dynamic Vehicle Testing & Perception Pipeline Integration |
| **Tire Dynamics & Slip** | Not Modeled (Idealized Rolling Assumption) | Basic 2D Rigid-Body Dynamics | Advanced Friction, Tire Slip, & Suspension Dynamics |
| **Sensor Simulation** | Ground-Truth Telemetry Output | 2D/3D Range Sensors & Odometry | Realistic Camera Pipelines, LiDAR, IMU, & Environment Contact |
| **Development Stage** | Initial Algorithm Prototyping & Fast Evaluation | Intermediate System Architecture Validation | Final Pre-Deployment System Integration & Testing |

## Reproduction guide:

### 1. Build the Workspace
```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Install build, simulation, and controller dependencies
sudo apt update && sudo apt install -y python3-colcon-common-extensions \
  python3-numpy python3-scipy ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-xacro

# Build the workspace (bicycle_sim, bicycle_control, track_environment)
cd /path/to/bicycle_gym-main
colcon build --symlink-install
source install/setup.bash
```

In every new terminal, source the ROS distribution and built workspace:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/bicycle_gym-main
source install/setup.bash
```
### 2. Launch Modes

| Mode | Launch Command | Section |
|---|---|---|
| **Interactive Keyboard Teleop** | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop` & `ros2 run teleop_twist_keyboard teleop_twist_keyboard` in another terminal
| **Lateral PID (Reactive)** | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=lateral_pid`
| **Pure Pursuit (Geometric Preview)**| `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=pure_pursuit`
| **Extended Kinematic MPC (Optimal Preview)** | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=mpc`

## Video

[demo video](./demo.mp4)
