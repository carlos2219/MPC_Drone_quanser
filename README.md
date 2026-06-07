# Model Predictive Control for Quanser QDrone2

## Project Overview

This project implements a hierarchical **Model Predictive Control (MPC)** system for autonomous flight of a Quanser QDrone2 quadrotor. The system decomposes the full 6-DOF control problem into three cascaded MPC layers: altitude control, attitude control, and planar (x-y) trajectory tracking. Each layer operates at its own prediction and sampling horizon, optimizing over finite prediction windows to compute optimal control sequences subject to physical actuator constraints.

## Motivation: MPC over PID and LQR

### Why MPC?

**PID controllers** are reactive (feedback-only) and struggle with:
- Actuator saturation and rate constraints
- Coupled multi-axis dynamics
- Aggressive setpoint changes requiring extensive tuning

**LQR** solves the infinite-horizon problem optimally but:
- Assumes unconstrained actuators
- Does not explicitly handle input/state bounds
- Cannot enforce physical limits (thrust, torque, angle constraints)

**MPC** combines the strengths of both:
1. **Explicit constraint handling** — saturations and limits are enforced during optimization, not after
2. **Finite-horizon optimality** — solves the constrained finite-horizon problem at every sample time
3. **Decoupling freedom** — layers can operate at different rates (altitude @ 4 Hz, attitude @ 40 Hz, lateral @ 4 Hz)
4. **Predictive authority** — uses future reference trajectory to anticipate and reduce overshoots
5. **Natural disturbance rejection** — gravity and model mismatch are explicitly modeled in the cost function

This project prioritizes **robustness to actuator limits and safe operation** over computational efficiency.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                        QUANSER QDRONE2                            │
│  6 DOF: x, y, z (position) + roll, pitch, yaw (attitude)         │
└──────────────────────────────────────────────────────────────────┘
                                  ↓
┌──────────────────────────────────────────────────────────────────┐
│              HIERARCHICAL MPC CONTROL STRUCTURE                   │
├──────────────────────────────────────────────────────────────────┤
│                                                                    │
│  Layer 1: ALTITUDE MPC (N=10, Ts=0.25s)                          │
│  ├─ Input:  z_cmd (commanded altitude)                           │
│  ├─ State:  [z, ż]ᵀ (position, velocity)                        │
│  ├─ Output: thrust_cmd (vertical force)                          │
│  └─ Constraint: 0 ≤ thrust ≤ 5.11 N                              │
│                                                                    │
│  Layer 2: ATTITUDE MPC (N=15, Ts=0.025s)                         │
│  ├─ Input:  φ_cmd, θ_cmd, ψ_cmd (roll, pitch, yaw setpoints)   │
│  ├─ State:  [φ, φ̇, θ, θ̇, ψ, ψ̇]ᵀ                                │
│  ├─ Output: τ_roll, τ_pitch, τ_yaw (torques)                     │
│  └─ Constraints: |τ| ≤ limits, |angle| ≤ π/4 (roll/pitch)        │
│                                                                    │
│  Layer 3: PLANAR MOTION MPC (N=15, Ts=0.25s)                     │
│  ├─ Input:  x_cmd, y_cmd (planar position references)           │
│  ├─ State:  [x, ẋ, y, ẏ]ᵀ                                        │
│  ├─ Output: φ_cmd, θ_cmd (attitude commands to Layer 2)         │
│  └─ Constraint: |φ|, |θ| ≤ π/4 (tilt angles)                     │
│                                                                    │
└──────────────────────────────────────────────────────────────────┘
                                  ↓
                        Motor Mixing Matrix
                    [Ω₁ Ω₂ Ω₃ Ω₄]ᵀ = M⁻¹[Fz τx τy τz]
                                  ↓
                    ┌─────────────────────┐
                    │   Motor Speed Cmds   │
                    │  to Motor Drivers    │
                    └─────────────────────┘
```

---

## MPC Formulation

### Discrete-Time Linear System Dynamics

Each layer uses a discretized linear model: **x[k+1] = Ax[k] + Bu[k] + Gd[k]**

where:
- **x[k]** = state vector (position, velocity, angles, angular rates)
- **u[k]** = control input (force/torque command)
- **d[k]** = disturbance (gravity, modeling error)
- **A, B, G** = discretized system matrices (via `c2d`)

#### State Vectors

**Altitude Layer:**
- x_z = [z, ż]ᵀ — vertical position and velocity

**Attitude Layer:**
- x_a = [φ, φ̇, θ, θ̇, ψ, ψ̇]ᵀ — Euler angles and angular velocities (decoupled axes)

**Planar Layer:**
- x_l = [x, ẋ, y, ẏ]ᵀ — horizontal position and velocity (decoupled x and y)

#### Measured Outputs

All three layers measure their respective states directly (full-state feedback). The system assumes:
- IMU provides φ, θ, ψ, φ̇, θ̇, ψ̇
- Barometer/GPS provides z, ż, x, ẋ, y, ẏ
- No filtering or state estimation is implemented (external responsibility)

### Cost Function (Quadratic Programming)

Each MPC layer minimizes:

```
J = Σ(k=0 to N-1) [‖y[k] - r[k]‖²_Q + ‖u[k]‖²_R + ‖u[k] - u[k-1]‖²_S]
```

where:
- **y[k]** = measured output (state subset)
- **r[k]** = reference output
- **u[k]** = control input
- **Q** = output tracking weight (penalizes deviation from reference)
- **R** = control effort weight (penalizes large actuations)
- **S** = rate-of-change weight (penalizes jerky inputs)
- **N** = prediction horizon (10–15 steps per layer)

#### Cost Function Matrices

**Altitude:** 
- Qy_z = 10 (high altitude tracking priority)
- Qu_z = 0.00001 (soft thrust saturation)

**Attitude:**
- Qy_a = diag([2500, 1500, 1500]) (aggressive roll/pitch stabilization)
- Qu_a = diag([250, 550, 500]) (torque penalization)

**Planar:**
- Qy_l = diag([100, 500]) (higher priority on y-tracking)
- Qu_l = diag([300, 800]) (strong input damping)

### Constraints

#### State Constraints (Hard Limits)

**Altitude:**
- 0 ≤ z ≤ ∞ (implicit: thrust ≥ 0)

**Attitude:**
- |φ| ≤ π/4 rad (±45°) — roll limit for quadrotor stability
- |θ| ≤ π/4 rad (±45°) — pitch limit
- |ψ| ≤ 100°/180°·π rad — yaw rate limit (secondary)

**Planar:**
- |x|, |y| ≤ ±2 m (workspace limits)

#### Input Constraints (Actuator Saturation)

**Altitude:**
- 0 ≤ thrust ≤ 5.11 N (motor thrust limit)

**Attitude:**
- |τ_roll|, |τ_pitch|, |τ_yaw| ≤ computed limits (derived from motor dynamics)
- Rate constraint: |Δτ| ≤ 0.1963 rad (rate-of-change limit per step)

**Planar:**
- |φ_cmd|, |θ_cmd| ≤ π/4 rad (commanded angle limits)
- Rate constraint: |Δφ_cmd|, |Δθ_cmd| ≤ 0.2 rad

### Prediction Horizons and Sampling Rates

| Layer    | Horizon (N) | Sample Time (Ts) | Horizon Duration |
|----------|-------------|------------------|------------------|
| Altitude | 10          | 0.25 s           | 2.5 s            |
| Attitude | 15          | 0.025 s          | 0.375 s          |
| Planar   | 15          | 0.25 s           | 3.75 s           |

**Design Rationale:**
- **Altitude** moves slowly (vertical dynamics) → longer horizon, slower sampling
- **Attitude** must react quickly (angular dynamics, gyro feedback) → short horizon, fast sampling
- **Planar** moderate speed → intermediate horizon and rate

---

## Mathematical Implementation Details

### Cost Function Construction (`Cost_Funct.m`)

The function builds the QP problem: **min ½uᵀHu + Fᵀu**

Outputs:
- **H** = 2(ΨᵀQΨ + Rₚ) — Hessian (quadratic term, 2× for QP convention)
- **F** = [F1, F2, F3, F4] — Linear coefficient vectors for constraints
  - F1: coupling with initial state
  - F2: coupling with reference trajectory
  - F3: input penalization
  - F4: gravity disturbance (altitude only)

where:
- **Φ** = state transition matrix (powers of A stacked)
- **Ψ** = input-to-state matrix (Toeplitz with B stacked)
- **C** = output selection matrix (extracts measured states)

### Constraint Construction (`Ineq_Calc.m`)

Builds inequality constraint matrices **Aineq·u ≤ G** from:
- Output bounds: y_min ≤ Cy ≤ y_max
- Input bounds: u_min ≤ u ≤ u_max
- Input rate bounds: Δu_min ≤ Δu ≤ Δu_max

Expands bounds across the prediction horizon (N copies of constraints).

### Input Constraint Expansion (`InConstraints.m`)

Simple helper that replicates a single constraint vector N times to create the horizon-wide constraint vector:
```
[u_max; u_max; ...; u_max]  (N copies)
```

### Motor Mixing (`Motor_Mapping_7_Inch.m`)

Inverse kinematics: desired thrust and torques → motor RPM commands

The motor matrix **M** maps:
```
[F_total]     [Ω₁]
[τ_roll  ]  = M [Ω₂]
[τ_pitch ] ·   [Ω₃]
[τ_yaw   ]     [Ω₄]
```

Geometry parameters (7-inch props):
- **L_Roll** = 254 mm (roll moment arm)
- **L_Pitch** = 203.2 mm (pitch moment arm)
- **K_Tau** = 68.9055 (yaw moment scaling)

### Reference Tracking Helper (`Pi_i.m`)

Selects the k-th state variable across the prediction horizon:
```
Pi_i(i, nx, N) ∈ ℝ^(nx × N·nx) — picks state i from stacked vector
```
Used to decouple tracking error calculation per timestep.

---

## System Parameters

### Drone Physical Properties

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| m | 1.504 | kg | Total mass (QDrone2) |
| g | 9.81 | m/s² | Gravity |
| Jxx | 0.01277 | kg·m² | Roll inertia |
| Jyy | 0.01337 | kg·m² | Pitch inertia |
| Jzz | 0.03047 | kg·m² | Yaw inertia |

### Motor / Actuator Properties

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| Kv | 2100 | RPM/V | Motor speed constant |
| Kt | 1/Kv | — | Motor torque constant |
| Tmax | 5.11 | N | Max thrust per motor |
| Imax | 5.82 | A | Max motor current |
| Tm | 0.04 | s | Motor time constant (actuation delay model) |

### Geometric Parameters

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| L_Roll | 254 | mm | Motor-to-motor distance (roll axis) |
| L_Pitch | 203.2 | mm | Motor-to-motor distance (pitch axis) |
| mRotor | 0.045 | kg | Mass of rotor + motor assembly |

---

## Dependencies

### Required MATLAB Toolboxes
- **Control System Toolbox** — `c2d` (continuous-to-discrete conversion)
- **Optimization Toolbox** — quadprog (QP solver)
- **Simulink** (if using Simulink models for real-time control)

### Hardware Interface
- **Quanser QDrone2** — quadrotor with onboard motor drivers and IMU
- **QUARC Real-Time Linux** — real-time environment for deterministic control loop
- **Joystick Interface** — for manual trajectory commands

### External Modules (if applicable)
- Sensor drivers (IMU, barometer, GPS)
- Motor driver firmware
- State estimator (Kalman filter, complementary filter)

---

## How to Run

### 1. Setup Environment

```matlab
% Open MATLAB in the project directory
cd /path/to/MPC_Drone_quanser

% Run setup script to initialize all MPC parameters
Setup_QDrone2_MPC
```

This script:
- Loads drone physical parameters (mass, inertia, motor constants)
- Discretizes continuous-time models for each layer
- Constructs cost function matrices (H, F1, F2, F3, F4)
- Constructs constraint matrices (Aineq, G1, G2, G3)
- Initializes motor mixing matrix
- Computes prediction horizons and sampling rates

### 2. Load Simulink Model (Real-Time Control)

```matlab
% Load the main control stack
load_system('QD2_DroneStack_CompleteMPC_2021a.slx')

% Compile and connect to QUARC hardware
rtwbuild('QD2_DroneStack_CompleteMPC_2021a')

% Run in real-time on the drone
```

The Simulink model implements:
- Three MPC blocks (altitude, attitude, planar) running asynchronously
- State feedback from sensors
- Reference trajectory input (from joystick or waypoint planner)
- Motor command output to ESC drivers

### 3. Command the Drone

#### Manual Control (Joystick)
```matlab
% Connect joystick input
% Sticks control: roll, pitch desired angles → planar MPC
%                 throttle → altitude MPC  
%                 yaw → yaw rate command
```

#### Autonomous Trajectory
```matlab
% Program waypoint references into the Simulink model
% MPC automatically tracks the reference with optimal inputs
```

### 4. Monitor Performance

- **Real-time plotting**: Position, velocity, angles, rates
- **Constraint violation detection**: Check if any limits are exceeded
- **Energy consumption**: Sum of motor commands over time
- **Computation time**: Monitor QP solver runtime per cycle

---

## Results and Demo

### Expected Behavior

✅ **Altitude Tracking**
- Setpoint changes tracked with minimal overshoot (< 10%)
- Smooth transition via MPC predictive authority
- Thrust command remains within [0, 5.11N]

✅ **Attitude Stabilization**
- Roll/pitch held within ±45° during aggressive maneuvers
- Yaw angle controlled independently
- Angular rates damped (no oscillations)

✅ **Planar Trajectory Tracking**
- x-y position follows reference with ~0.1 m steady-state error
- Velocity outputs smooth; minimal jerk
- Attitude commands from planar MPC remain saturated within limits

✅ **Constraint Satisfaction**
- All state and input constraints honored throughout flight
- Graceful saturation prevents unstable control commands
- Rate limiters prevent actuator chattering

### Performance Metrics

| Metric | Target | Achieved |
|--------|--------|----------|
| Altitude RMSE | < 0.2 m | — |
| Planar position RMSE | < 0.15 m | — |
| Attitude overshoot | < 15% | — |
| QP solve time (per cycle) | < 5 ms | — |
| Constraint violations | 0 | — |

*Placeholders for empirical results from flight test.*

### Flight Test Videos / Data

- **Video 1**: Hovering altitude hold with 0.5 m step input
- **Video 2**: Figure-8 trajectory in x-y plane
- **Video 3**: Aggressive yaw maneuver with roll/pitch stabilization
- **Data 1**: Logged telemetry (position, velocity, attitudes, motor commands)

*To be populated with actual flight test results.*

---

## Code Structure

```
MPC_Drone_quanser/
├── README.md                              # This file
├── ARCHITECTURE.md                        # Control loop diagram and block structure
├── Setup_QDrone2_MPC.m                    # Main initialization script
├── Cost_Funct.m                           # MPC cost function construction
├── Ineq_Calc.m                            # Constraint matrix assembly
├── InConstraints.m                        # Horizon-wise constraint expansion
├── Pi_i.m                                 # State selection helper
├── Motor_Mapping_7_Inch.m                 # Inverse kinematics (thrust/torque → RPM)
├── QD2_DroneStack_CompleteMPC_2021a.slx   # Main Simulink control model
├── QD2_MissionCtrl_Pipas1.slx             # Mission planner variant 1
├── QD2_MissionCtrl_Pipas2.slx             # Mission planner variant 2
└── LICENSE                                # License information
```

---

## Extending the Controller

### Adding a Waypoint Planner
Modify reference input r[k] to step through GPS waypoints with smooth interpolation.

### Implementing Disturbance Rejection
Add wind model to cost function (state-dependent disturbance) or implement Kalman filter to estimate persistent biases.

### Tuning MPC Weights
Adjust **Qy** (output weight) and **Qu** (input weight) in `Setup_QDrone2_MPC.m`:
- Higher Qy → tighter tracking (faster, more oscillatory)
- Higher Qu → smoother inputs (slower, larger overshoot)

### Reconfiguring Horizons
Modify **N** (prediction horizon) and **Ts** (sampling time) per layer:
- Longer N → better preview, higher compute cost
- Shorter Ts → faster response, but requires better sensors and actuators

---

## References

- **MPC Theory**: Boyd & Parikh, "Convex Optimization" (constrained QP)
- **Quadrotor Dynamics**: Beard & McLain, "Small Unmanned Aircraft: Theory and Practice"
- **Quanser Documentation**: QDrone2 Hardware Manual & Control Architecture Guide
- **Discrete Control**: Franklin, Powell, Workman, "Digital Control of Dynamic Systems"

---

## Authors

- **Control Design**: [Your Name / Team]
- **Implementation**: Simulink + MATLAB MPC
- **Hardware Platform**: Quanser QDrone2
- **Institution**: ITESM (Instituto Tecnológico y de Estudios Superiores de Monterrey)

---

## License

See `LICENSE` file.

---

**Last Updated**: June 6, 2026
