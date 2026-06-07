# Quick Start Guide: QDrone2 MPC Control System

A step-by-step guide to get the hierarchical Model Predictive Control system running on a Quanser QDrone2.

---

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Configuration & Setup](#configuration--setup)
4. [Running the System](#running-the-system)
5. [Manual Flight Testing](#manual-flight-testing)
6. [Autonomous Mission](#autonomous-mission)
7. [Monitoring & Debugging](#monitoring--debugging)
8. [Troubleshooting](#troubleshooting)

---

## Prerequisites

### Software Requirements
- **MATLAB R2021a or later** (Control System Toolbox, Optimization Toolbox)
- **Simulink**
- **QUARC Real-Time Control** (Linux target for QDrone2)
- **Git** (for cloning/updating repository)

### Hardware Requirements
- **Quanser QDrone2** quadrotor
- **Computer with real-time Linux kernel** (or dual-boot setup)
- **Joystick** (PS4 controller, Xbox, or compatible)
- **USB-to-serial cable** (for debugging)

### Optional Tools
- **MATLAB Coder** (if modifying MPC algorithms)
- **Simulink Coder** (for custom code generation)
- **MATLAB Support for Arduino** (if extending to other platforms)

### Check Your Setup
```matlab
% In MATLAB command window:
ver                           % Check installed toolboxes
which quadprog                % Verify Optimization Toolbox
which c2d                      % Verify Control System Toolbox
```

---

## Installation

### 1. Clone the Repository
```bash
cd ~/projects/                                    % Or your preferred directory
git clone https://github.com/carlos2219/MPC_Drone_quanser.git
cd MPC_Drone_quanser
```

### 2. Verify Files
```bash
ls -la *.m *.slx README.md ARCHITECTURE.md
# Should show:
# - Setup_QDrone2_MPC.m
# - Cost_Funct.m
# - Ineq_Calc.m
# - InConstraints.m
# - Pi_i.m
# - Motor_Mapping_7_Inch.m
# - QD2_DroneStack_CompleteMPC_2021a.slx
# - (other Simulink models)
```

### 3. Open in MATLAB
```matlab
% Add project to MATLAB path
addpath(genpath('/path/to/MPC_Drone_quanser'))

% Open the repo
cd /path/to/MPC_Drone_quanser
```

---

## Configuration & Setup

### Step 1: Review Parameters (Optional)
Before running, you may want to customize the MPC tuning. Edit **Setup_QDrone2_MPC.m**:

```matlab
% Altitude layer weights (line ~56-57)
Qy_z = 10;        % Higher = tighter altitude tracking
Qu_z = 0.00001;   % Input effort (very soft)

% Attitude layer weights (line ~86-87)
Qy_a = [2500 0 0; 0 1500 0; 0 0 1500];  % Roll, pitch, yaw priorities
Qu_a = [250 0 0; 0 550 0; 0 0 500];     % Torque effort

% Planar layer weights (line ~130-131)
Qy_l = [100 0; 0 500];      % X, Y position priorities
Qu_l = [300 0; 0 800];      % Angle command effort
```

**Tuning Guide:**
- **Increase Qy** → tighter tracking, faster response (risk: overshoot)
- **Increase Qu** → smoother inputs, slower response (safer)
- Keep **asymmetric weights** if hardware is asymmetric (e.g., y-heavy)

### Step 2: Initialize MPC Parameters

Open MATLAB and run the setup script:

```matlab
% Open MATLAB and navigate to repo
cd /path/to/MPC_Drone_quanser

% Execute setup script (this populates Simulink workspace)
Setup_QDrone2_MPC

% Verify variables are loaded
whos H_z F1_z H_a H_l          % Should show matrices in workspace
```

**What this does:**
- Loads drone physical parameters (mass, inertia, motor constants)
- Discretizes 3 continuous-time models (altitude, attitude, planar)
- Pre-computes cost function matrices (H, F1, F2, F3, F4) for QP solver
- Pre-computes constraint matrices (Aineq, G1, G2, G3)
- Loads motor mixing matrix for inverse kinematics

### Step 3: Load Simulink Model

```matlab
% Load main control model
load_system('QD2_DroneStack_CompleteMPC_2021a.slx')

% Verify setup completed
assert(exist('H_z','var')==1, 'MPC matrices not loaded! Run Setup_QDrone2_MPC first')
```

---

## Running the System

### Option A: Simulation (No Hardware)

For testing control logic without flying:

```matlab
% Set simulation parameters
set_param('QD2_DroneStack_CompleteMPC_2021a', ...
    'SimulationMode', 'normal', ...
    'SolverName', 'ode45', ...
    'StopTime', '10')    % 10 seconds simulation time

% Run simulation
sim('QD2_DroneStack_CompleteMPC_2021a')

% Plot results
plot(ans.simout)       % Shows position, velocity, angles over time
```

**Useful plots:**
- `altitude_log` — vertical position tracking
- `attitude_log` — roll/pitch/yaw angles
- `planar_log` — x-y trajectory
- `motor_cmds` — motor speed commands (diagnostics)

---

### Option B: Real-Time on Hardware

#### 1. **Prepare Real-Time System**

Ensure QUARC is installed and configured:
```bash
# On real-time Linux machine
quarc_linux_console -status   # Check QUARC daemon

# Configure network (if remote target)
# Edit QUARC settings for target machine IP
```

#### 2. **Build Real-Time Code**

```matlab
% In MATLAB (on development machine)
rtwbuild('QD2_DroneStack_CompleteMPC_2021a')

% Wait for compilation... (2-5 minutes)
% Output: QD2_DroneStack_CompleteMPC_2021a.rt-linux_qdrone2
```

#### 3. **Connect Hardware**

```matlab
% Target configuration
set_param('QD2_DroneStack_CompleteMPC_2021a', ...
    'RTWCompilerOptimizationLevel', 3, ...          % Max optimization
    'SystemTargetFile', 'quarc_linux.tlc')          % Linux target

% Run on real-time target
rtwbuild('QD2_DroneStack_CompleteMPC_2021a')
```

#### 4. **Monitor in Real-Time**

```matlab
% Open QUARC Control Center (GUI)
quarc_control

% Select model: QD2_DroneStack_CompleteMPC_2021a.rt-linux_qdrone2
% Click "Download & Run"
% Observe real-time signals in QUARC oscilloscope
```

---

## Manual Flight Testing

### Step 1: Arm the Drone

```matlab
% Pre-flight checklist:
% ✓ Battery charged (>11V)
% ✓ All props on and secure
% ✓ Props clear of obstacles
% ✓ Joystick connected and calibrated

% In QUARC Control Center:
% 1. Set "Arming" slider to ON
% 2. Check LED feedback (solid green = ready)
% 3. Confirm thrust command is zero
```

### Step 2: Manual Control

#### Joystick Mapping (Default):
```
LEFT STICK:
  • Up/Down    → Altitude setpoint (z_cmd)
  • Left/Right → Yaw angle rate

RIGHT STICK:
  • Up/Down    → Pitch angle command (θ_cmd)
  • Left/Right → Roll angle command (φ_cmd)
```

#### First Flight:
```matlab
% Start at ZERO throttle
% Gently increase altitude command (left stick up)
%   → Drone should smoothly climb
%   → MPC optimizes thrust command, respects saturation limits

% Test roll (right stick right)
%   → Drone tilts right, MPC stabilizes attitude
%   → Should not oscillate or become unstable

% Test pitch (right stick up)
%   → Drone tilts forward, maintains altitude
%   → MPC couples altitude with attitude

% Test yaw (left stick left)
%   → Drone rotates left axis
%   → Independent of pitch/roll
```

#### Expected Behavior:
- **Smooth response** — no oscillations or chattering
- **Stable hovering** — maintains altitude when sticks centered
- **Rate limiting** — commands don't change abruptly
- **Constraint satisfaction** — no jerky saturations

### Step 3: Telemetry Logging

```matlab
% Enable data logging in Simulink model
% Signals to log:
%   - z, ż (altitude)
%   - φ, θ, ψ (angles)
%   - x, y, ẋ, ẏ (planar position/velocity)
%   - F_z, τ_roll, τ_pitch, τ_yaw (commands)

% Save to MATLAB workspace
set_param('QD2_DroneStack_CompleteMPC_2021a', ...
    'SaveFormat', 'Array', ...
    'SignalLogging', 'on')

% After flight, analyze:
loganalyzer(logsout)   % Built-in log viewer
```

---

## Autonomous Mission

### Option 1: Waypoint Tracking

```matlab
% Define waypoints [x, y, z, psi] (meters, radians)
waypoints = [
    0.0,  0.0, 1.0, 0;          % Start (1m altitude)
    1.0,  0.0, 1.0, 0;          % Right
    1.0,  1.0, 1.0, 0;          % Forward
    0.0,  1.0, 1.5, pi/2;       % Left, higher altitude
    0.0,  0.0, 1.0, 0;          % Return home
];

% Compute smooth trajectory through waypoints
% (Quintic polynomial interpolation recommended)

% Load into Simulink model
assignin('base', 'waypoint_sequence', waypoints)
```

### Option 2: Circle Trajectory

```matlab
% Circle in x-y plane, constant altitude
t = 0:0.01:10;                  % 10 seconds
radius = 1.0;                   % 1 meter
x_ref = radius * cos(2*pi*t/10);
y_ref = radius * sin(2*pi*t/10);
z_ref = 1.0 * ones(size(t));    % Constant altitude

assignin('base', 'ref_trajectory', [x_ref; y_ref; z_ref])
```

### Option 3: Figure-8 Trajectory

```matlab
% Lemniscate (figure-8 shape)
t = 0:0.01:20;
scale = 0.5;
x_ref = scale * cos(t) / (1 + sin(t).^2);
y_ref = scale * sin(t) .* cos(t) / (1 + sin(t).^2);
z_ref = 1.0 * ones(size(t));

% Run autonomous mission
sim('QD2_DroneStack_CompleteMPC_2021a')
```

### Step 4: Execute Mission

```matlab
% In QUARC Control Center:
% 1. Set mode to "AUTONOMOUS" (switch parameter)
% 2. Click "Download & Run"
% 3. Observe drone tracks reference trajectory
% 4. Monitor MPC solver time (should be <5ms per cycle)
% 5. Log data for post-flight analysis
```

---

## Monitoring & Debugging

### Real-Time Monitoring

**QUARC Control Center Dashboard:**
```
Signal: z                  Current: 1.23 m   [Graph]
Signal: altitude_error     Current: 0.05 m   [Graph]
Signal: F_thrust           Current: 14.8 N   [Graph]
Signal: solver_time_ms     Current: 3.2 ms   [Graph]
```

**Key signals to watch:**
- `solver_time_ms` — QP solver runtime per cycle
  - Altitude: should be <5 ms @ 4 Hz
  - Attitude: should be <3 ms @ 40 Hz
  - Planar: should be <5 ms @ 4 Hz
- `constraint_violations` — any output > 0 = problem
- `motor_saturation` — count of saturated motors

### Post-Flight Analysis

```matlab
% Load logged data
load flight_data.mat             % Or import from QUARC

% Plot altitude tracking
figure; subplot(2,1,1)
plot(t, z_measured, 'b', t, z_reference, 'r--')
xlabel('Time (s)'); ylabel('Altitude (m)')
legend('Measured', 'Reference')

% Plot tracking error
subplot(2,1,2)
plot(t, z_reference - z_measured, 'r')
xlabel('Time (s)'); ylabel('Error (m)')
title(sprintf('RMSE: %.3f m', rms(z_reference - z_measured)))
```

### Debugging MPC Issues

#### Problem: Oscillations
```matlab
% Solution: Increase Qu (input weight)
% Increase damping in cost function
Qu_a = [500 0 0; 0 1000 0; 0 0 1000];  % Was [250, 550, 500]
Setup_QDrone2_MPC                       % Recompute matrices
```

#### Problem: Slow Response
```matlab
% Solution: Increase Qy (output weight)
Qy_a = [5000 0 0; 0 3000 0; 0 0 3000]; % Increase tracking priority
Setup_QDrone2_MPC
```

#### Problem: Saturation/Jerky Commands
```matlab
% Solution: Loosen constraints or increase horizon N
N_a = 20;        % Was 15 steps
ts_a = 0.025;    % Keep same sample time
Setup_QDrone2_MPC
```

#### Problem: Long Solver Runtime
```matlab
% Solution: Reduce horizon (faster but less optimal)
N_a = 10;        % Was 15 steps
Setup_QDrone2_MPC

% Or: Reduce prediction horizon at expense of preview
N_a = 12;
N_z = 8;
N_l = 12;
Setup_QDrone2_MPC
```

---

## Troubleshooting

### MATLAB Issues

| Issue | Solution |
|-------|----------|
| `undefined function c2d` | Install Control System Toolbox: `ver` → verify installation |
| `undefined function quadprog` | Install Optimization Toolbox |
| `H_z not found in workspace` | Run `Setup_QDrone2_MPC` first |
| `Simulink model won't load` | Check MATLAB version (R2021a+) and file path |
| `Memory error during sim` | Reduce logging, shorten simulation time |

### QUARC/Hardware Issues

| Issue | Solution |
|-------|----------|
| `QUARC daemon not running` | `quarc_linux_console -start` on real-time machine |
| `Connection refused` | Check IP/hostname, verify network, restart daemon |
| `Model download fails` | Run `rtwbuild` again, check compiler logs |
| `Real-time deadline missed` | Reduce MPC horizon or increase sample time |
| `Drone doesn't respond` | Check USB connection, verify serial port in QUARC |

### Control Issues

| Issue | Solution |
|-------|----------|
| Drone drifts in hover | Check barometer calibration, verify wind estimates |
| Unstable roll/pitch | Increase Qu_a values (more damping) |
| Altitude overshoots | Reduce Qy_z (less aggressive) |
| Jerky trajectory | Increase Qu_l, enable smoothing filter on references |

### Performance Troubleshooting

```matlab
% Check MPC solver convergence
figure; 
plot(t, solver_status)          % Should all be = 0 (optimal)
ylim([-1 2])
xlabel('Time (s)'); ylabel('QP Status')
legend('0=optimal, 1=iterating, 2=infeasible')
```

---

## Next Steps

### After Successful First Flight:

1. **Tune for your hardware**
   - Adjust Qy, Qu based on actual response
   - Log multiple flights, analyze RMSE
   - Fine-tune constraint limits based on motor behavior

2. **Add State Estimation**
   - Implement Kalman filter for sensor fusion
   - Estimate wind disturbances
   - Handle GPS dropouts gracefully

3. **Extend Control Scope**
   - Add obstacle avoidance (visual servoing)
   - Implement adaptive control (online learning)
   - Deploy on swarm (multi-drone coordination)

4. **Document Results**
   - Collect flight test data
   - Generate performance plots for thesis
   - Create comparison vs baseline (PID/LQR)

---

## Additional Resources

### Documentation
- **README.md** — Project overview and motivation
- **ARCHITECTURE.md** — Detailed control architecture diagrams
- **Quanser QDrone2 Manual** — Hardware specifications
- **MATLAB MPC Toolbox Docs** — Advanced tuning

### Code References
- `Setup_QDrone2_MPC.m` — Parameter initialization and discretization
- `Cost_Funct.m` — MPC cost function (Hessian, linear terms)
- `Ineq_Calc.m` — Constraint matrix construction
- `Motor_Mapping_7_Inch.m` — Thrust/torque to motor RPM conversion

### External Links
- [Quanser Support Portal](https://www.quanser.com)
- [QUARC Documentation](https://www.quanser.com/quarc)
- [MATLAB Control System Toolbox](https://www.mathworks.com/products/control.html)
- [Boyd & Parikh: Convex Optimization](https://web.stanford.edu/~boyd/cvxbook/)

---

## Common Commands Reference

```matlab
% Setup and initialize
Setup_QDrone2_MPC                  % Load all MPC parameters

% Simulate
sim('QD2_DroneStack_CompleteMPC_2021a')

% Build for real-time
rtwbuild('QD2_DroneStack_CompleteMPC_2021a')

% Monitor real-time execution
quarc_control                      % Open QUARC GUI

% Log data
logsout                            % Access logged signals (if enabled)

% Analyze flight data
loganalyzer(logsout)              % Interactive log viewer

% Update parameters
Qy_z = 20;                        % Change weight
Setup_QDrone2_MPC                 % Recompute MPC matrices
sim(...)                          % Test with new tuning
```

---

## Support & Feedback

- **Issues?** Check ARCHITECTURE.md for system overview
- **Tuning questions?** Review README.md "Tuning Parameters" section
- **Code questions?** Check docstrings in MATLAB files (e.g., `help Cost_Funct`)
- **Hardware issues?** Consult Quanser documentation

---

**Last Updated:** June 7, 2026  
**For:** Quanser QDrone2 with MATLAB/Simulink + QUARC Real-Time Control  
**Author:** Control Systems Team  
**Institution:** ITESM
