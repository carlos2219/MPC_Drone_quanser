# MPC Control Architecture for QDrone2

## Overview

This document describes the hierarchical, multi-rate control architecture used in the QDrone2 MPC system. The architecture decomposes the 6-DOF regulation and trajectory tracking problem into three independent, cascaded MPC layers, each optimizing at its own rate and horizon.

---

## Control Loop Diagram

### Overall System Flow

```
                        ┌─────────────────────────────────────────┐
                        │   REFERENCE TRAJECTORY GENERATOR         │
                        │   (Waypoints, mission planner, joystick) │
                        └──────────────────┬──────────────────────┘
                                           │
                    ┌──────────────────────┴──────────────────────┐
                    │  r_z(t): altitude setpoint                   │
                    │  r_x(t), r_y(t): planar position setpoints   │
                    │  r_ψ(t): yaw angle setpoint                  │
                    └──────────────────────┬──────────────────────┘
                                           │
                    ┌──────────────────────┴──────────────────────┐
                    │                                               │
          ┌─────────▼─────────┐                       ┌─────────────▼────────┐
          │  ALTITUDE MPC     │                       │  PLANAR MOTION MPC   │
          │   (4 Hz, N=10)    │                       │   (4 Hz, N=15)       │
          │                   │                       │                      │
          │ z_ref → [F_z]    │                       │ [x_ref, y_ref] →     │
          │                   │                       │   [φ_cmd, θ_cmd]    │
          └─────────┬─────────┘                       └──────────┬───────────┘
                    │                                           │
                    │ F_thrust [0, 5.11] N                     │ φ_cmd, θ_cmd [-π/4, π/4]
                    │                                           │
                    │              ┌──────────────────────────┬─┘
                    │              │  ψ_ref → [ψ̇_cmd]       │
                    │              │                          │
                    │              │    ┌────────────────────▼──────┐
                    │              │    │  ATTITUDE MPC             │
                    │              │    │   (40 Hz, N=15)           │
                    │              │    │                           │
                    │              │    │ [φ_cmd, θ_cmd, ψ_cmd] →  │
                    │              │    │   [τ_roll, τ_pitch, τ_yaw]│
                    │              │    └────────────┬──────────────┘
                    │              │                 │
                    │              │  τ_roll, τ_pitch, τ_yaw
                    │              │  F_thrust
                    │              │
                    └──────────────┴────────┬────────────────────────┐
                                            │
                        ┌───────────────────▼──────────────────┐
                        │  MOTOR MIXING MATRIX                 │
                        │  M: thrust/torque → motor RPM        │
                        │                                       │
                        │  [Ω₁, Ω₂, Ω₃, Ω₄]ᵀ = M⁻¹[Fz τx τy τz]
                        └───────────────────┬──────────────────┘
                                            │
                    ┌───────────┬───────────┼───────────┬──────────┐
                    │           │           │           │          │
               ┌────▼──┐  ┌─────▼──┐  ┌─────▼──┐  ┌──────▼─┐
               │ Motor 1│  │ Motor 2│  │ Motor 3│  │ Motor 4│
               │ Driver │  │ Driver │  │ Driver │  │ Driver │
               └────┬───┘  └────┬───┘  └────┬───┘  └───┬────┘
                    │           │           │          │
                    └───────────┬───────────┴──────────┘
                                │
                        ┌───────▼─────────┐
                        │  QUADROTOR      │
                        │  (Plant)        │
                        │  6-DOF Dynamics │
                        └───────┬─────────┘
                                │
          ┌─────────────────────┼─────────────────────┐
          │                     │                     │
    ┌─────▼────┐        ┌──────▼──────┐       ┌──────▼──────┐
    │   IMU    │        │  Barometer  │       │   GPS/Pos   │
    │          │        │  (z, ż)     │       │   (x, y)    │
    │ angles & │        │             │       │             │
    │ rates    │        │             │       │             │
    └─────┬────┘        └──────┬──────┘       └──────┬──────┘
          │                    │                     │
          └────────────────┬───┴─────────────────────┘
                           │
              ┌────────────▼──────────────┐
              │  STATE FEEDBACK           │
              │  (Full state measured)    │
              │  No filtering assumed     │
              └────────────┬──────────────┘
                           │
          ┌────────────────┴────────────────────┐
          │   [x, ẋ, y, ẏ] (planar)           │
          │   [φ, φ̇, θ, θ̇, ψ, ψ̇] (attitude)  │
          │   [z, ż] (altitude)                │
          └────────────────┬────────────────────┘
                           │
                           └─────────┐
                                     │ (Feedback loop)
                                     │ (back to MPC blocks)
                                     ▼
```

---

## Detailed Block Diagram: MPC Solver Pipeline

Each MPC layer follows the same computational structure at its own rate:

```
┌──────────────────────────────────────────────────────────────────┐
│                   MPC OPTIMIZATION CYCLE (per Ts)                │
├──────────────────────────────────────────────────────────────────┤
│                                                                    │
│  1. READ STATE                                                    │
│     x[k] ← [sensor measurement]                                   │
│                                                                    │
│  2. READ REFERENCE                                                │
│     r[0:N-1] ← [trajectory generator]                             │
│                                                                    │
│  3. BUILD QP PROBLEM (offline, or recompute)                     │
│                                                                    │
│     ┌────────────────────────────────────────┐                    │
│     │  min ½uᵀHu + Fᵀu                      │                    │
│     │  subject to:                           │                    │
│     │    Aineq·u ≤ G  (state/input bounds)   │                    │
│     │    u ∈ ℝ^(N·nu)  (N-step control seq) │                    │
│     └────────────────────────────────────────┘                    │
│                                                                    │
│     H, F computed by Cost_Funct.m                                 │
│     Aineq, G computed by Ineq_Calc.m                              │
│                                                                    │
│  4. SOLVE QP                                                      │
│     u* ← quadprog(H, F, Aineq, G)  (MATLAB Optimization Toolbox) │
│                                                                    │
│  5. EXTRACT FIRST CONTROL ACTION                                  │
│     u[k] ← u*[1:nu]  (first nu elements of solution)             │
│                                                                    │
│  6. APPLY COMMAND                                                │
│     [command output] ← saturation( u[k] )                        │
│                                                                    │
└──────────────────────────────────────────────────────────────────┘
         │ (repeats every Ts)
         │
         ├─ Altitude:  Ts = 0.25 s  (4 Hz)
         ├─ Attitude:  Ts = 0.025 s (40 Hz)
         └─ Planar:    Ts = 0.25 s  (4 Hz)
```

---

## Hierarchical Control Layers

### Layer 1: Altitude Control (Vertical Dynamics)

```
    z_cmd (reference altitude)
         │
         ▼
    ┌────────────────────────┐
    │  ALTITUDE MPC          │
    │                        │
    │  State: x_z = [z, ż]   │
    │  Control: u_z = F_z    │
    │  Meas: [z, ż]          │
    │                        │
    │  min Σ(‖z[k]-z_ref‖²_Qy │
    │      +‖F_z[k]‖²_Qu)    │
    │                        │
    │  subject to:           │
    │    0 ≤ F_z ≤ 5.11 N    │
    │                        │
    │  N = 10, Ts = 0.25s    │
    └────────┬───────────────┘
             │
             ▼
         F_thrust [N]


SYSTEM MODEL:
┌─────────────────────────────────────────┐
│  ż̇ = F_z/m - g                         │
│                                         │
│  Discretized: [z, ż]^T → c2d(Ts=0.25) │
│                                         │
│  Gravity modeled as constant            │
│  disturbance G in state equation        │
└─────────────────────────────────────────┘

PARAMETERS:
  m = 1.504 kg (total mass)
  Qy_z = 10 (high altitude tracking priority)
  Qu_z = 0.00001 (soft thrust limiting)
  g = -9.81 m/s² (gravity disturbance)
```

**Why separate altitude from attitude?**
- Altitude (vertical) dynamics are slower and decoupled from roll/pitch
- Can use longer prediction horizon (N=10) with slower sampling (Ts=0.25s)
- Reduces computation burden vs. full 6-DOF MPC
- Altitude reference updates less frequently than attitude corrections

---

### Layer 2: Attitude Control (Rotational Dynamics)

```
  φ_cmd, θ_cmd, ψ_cmd (attitude setpoints from planar MPC or joystick)
         │
         ▼
    ┌────────────────────────────────────────┐
    │  ATTITUDE MPC                          │
    │                        │               │
    │  State: x_a = [φ φ̇ θ θ̇ ψ ψ̇]ᵀ        │
    │  Control: u_a = [τx τy τz]ᵀ           │
    │  Meas: [φ φ̇ θ θ̇ ψ ψ̇]ᵀ  (IMU)       │
    │                                        │
    │  min Σ(‖[φ φ θ ψ][k]-ref‖²_Qy         │
    │      +‖[τx τy τz][k]‖²_Qu)            │
    │                                        │
    │  subject to:                           │
    │    |φ|, |θ| ≤ π/4 rad                 │
    │    |τ*| ≤ limits (motor constraints)  │
    │    |Δτ*| ≤ 0.1963 rad (rate limit)    │
    │                                        │
    │  N = 15, Ts = 0.025s (40 Hz)          │
    └────────┬─────────────────────────────┘
             │
             ▼
    [τ_roll, τ_pitch, τ_yaw]


SYSTEM MODEL (DECOUPLED AXES):
┌───────────────────────────────────────────────────┐
│  φ̈ = τ_roll / Jxx                               │
│  θ̈ = τ_pitch / Jyy                              │
│  ψ̈ = τ_yaw / Jzz                                │
│                                                   │
│  Discretized per axis: c2d(Ts=0.025)             │
│                                                   │
│  Linear model assumes small angles               │
│  (valid for |φ|, |θ| < π/4)                     │
└───────────────────────────────────────────────────┘

PARAMETERS:
  Jxx = 0.01277 kg⋅m² (roll inertia)
  Jyy = 0.01337 kg⋅m² (pitch inertia)
  Jzz = 0.03047 kg⋅m² (yaw inertia)
  Qy_a = diag([2500, 1500, 1500]) (aggressive tracking)
  Qu_a = diag([250, 550, 500]) (torque penalization)
```

**Why fast attitude loop?**
- Rotational dynamics are 10× faster than translation
- IMU feedback is high-bandwidth (gyroscope rates available at >100 Hz)
- Attitude stabilization requires fast feedback to suppress oscillations
- Short horizon (N=15 steps @ 40 Hz = 0.375 s) sufficient for local stabilization

**Attitude constraints:**
- Roll/pitch limited to ±45° to maintain hovering efficiency (full vertical thrust available)
- Beyond ±45°, drone loses vertical thrust and may fall
- Yaw has no physical limit but rate-limited for motor smoothness

---

### Layer 3: Planar Motion Control (Horizontal Trajectory)

```
  x_cmd, y_cmd (reference positions from mission planner)
         │
         ▼
    ┌────────────────────────────────┐
    │  PLANAR MOTION MPC             │
    │                                │
    │  State: x_l = [x ẋ y ẏ]ᵀ      │
    │  Control: u_l = [φ_cmd θ_cmd]ᵀ│
    │  Meas: [x ẋ y ẏ]ᵀ (GPS/sensor)│
    │                                │
    │  min Σ(‖[x y][k]-ref‖²_Qy     │
    │      +‖[φ θ][k]‖²_Qu)         │
    │                                │
    │  subject to:                   │
    │    |φ_cmd|, |θ_cmd| ≤ π/4 rad │
    │    |x|, |y| ≤ ±2 m (workspace)│
    │    |Δφ|, |Δθ| ≤ 0.2 rad       │
    │                                │
    │  N = 15, Ts = 0.25s (4 Hz)    │
    └────────┬───────────────────────┘
             │
             ├─→ φ_cmd (roll command to Attitude MPC)
             └─→ θ_cmd (pitch command to Attitude MPC)


SYSTEM MODEL (GRAVITY-DRIVEN, SMALL ANGLE):
┌─────────────────────────────────────────────────┐
│  ẍ = g·sin(φ) ≈ g·φ  (for small φ)             │
│  ÿ = -g·sin(θ) ≈ -g·θ (for small θ)            │
│                                                 │
│  Decoupled x and y axes                        │
│  Discretized: [x ẋ y ẏ]^T → c2d(Ts=0.25)      │
│                                                 │
│  φ, θ are control inputs (not states)          │
└─────────────────────────────────────────────────┘

PARAMETERS:
  Qy_l = diag([100, 500]) (y-tracking higher priority)
  Qu_l = diag([300, 800]) (strong damping on angle commands)
  Workspace: x, y ∈ [-2, 2] m
```

**Why separate planar from altitude?**
- Horizontal and vertical dynamics decouple in small-angle regime
- Planar motion slower than attitude stabilization
- Can share longer horizon (N=15 @ 0.25s = 3.75s) with altitude
- Allows attitude loop to stabilize while planar loop plans ahead

**Control flow:**
1. Planar MPC outputs φ_cmd, θ_cmd (desired roll/pitch angles)
2. These become setpoints for Attitude MPC
3. Attitude MPC controls actual angles to match commands
4. Actual tilting produces acceleration → planar motion

---

## Sensor-to-Control Feedback Paths

```
┌──────────────────────────────────────────────────────────┐
│                    IMU (Gyroscope)                        │
│           [φ̇, θ̇, ψ̇]ᵀ @ 100+ Hz                        │
│                         │                                 │
│         ┌───────────────┼───────────────┐                │
│         │               │               │                 │
│    ┌────▼────┐    ┌────▼────┐    ┌─────▼────┐            │
│    │ Gyro LPF│    │ Integrate│    │Integrate │            │
│    │         │    │ w/bias   │    │ w/bias   │            │
│    └────┬────┘    └────┬────┘    └────┬─────┘            │
│         │               │               │                 │
│         ▼               ▼               ▼                 │
│      φ̇ (measured)   θ̇ (measured)   ψ̇ (measured)       │
│         │               │               │                 │
│    ┌────▼────────────────┴───────────────┘                │
│    │        [φ φ̇ θ θ̇ ψ ψ̇]ᵀ                            │
│    │    (Attitude State @ 40 Hz)                          │
│    └────┬─────────────────────────────┘                   │
│         │                                                  │
│         └──→ [Attitude MPC] (input: measured state)       │
│                                                            │
└──────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────┐
│                  BAROMETER (Altimeter)                    │
│              z (altitude) @ 4-10 Hz                       │
│              ż ≈ z[k] - z[k-1] / Ts                      │
│                         │                                 │
│             ┌───────────┴──────────┐                     │
│             │   [z, ż]ᵀ            │                     │
│             │(Altitude State @ 4 Hz)│                    │
│             └───────────┬──────────┘                     │
│                         │                                 │
│                    [Altitude MPC]                         │
└──────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────┐
│               GPS / POSITION SENSOR                       │
│             [x, y] @ 4-10 Hz                             │
│             [ẋ, ẏ] from differentiation                  │
│                         │                                 │
│             ┌───────────┴──────────┐                     │
│             │ [x ẋ y ẏ]ᵀ           │                     │
│             │(Planar State @ 4 Hz) │                     │
│             └───────────┬──────────┘                     │
│                         │                                 │
│                  [Planar Motion MPC]                      │
└──────────────────────────────────────────────────────────┘
```

---

## Computational Timeline: Multi-Rate Execution

```
Time (s)    Altitude MPC    Attitude MPC        Planar MPC      Notes
            (4 Hz)          (40 Hz)             (4 Hz)
────────────────────────────────────────────────────────────────────
0.000       SOLVE           SOLVE               SOLVE           t=0: all start
0.001       ─               │
0.002       ─               SOLVE
0.003       ─               │
0.004       ─               SOLVE
0.005       ─               │
...         ─               ... (8× per 0.04s)
0.024       ─               SOLVE
0.025       ─               SOLVE               ─               40 Hz tick (attitude)
0.026       ─               │
...         ─               ... (continues)
0.050       ─               SOLVE               ─               next 40 Hz
0.075       ─               SOLVE               ─
0.100       SOLVE           SOLVE               ─               25× Attitude = 1× Altitude
0.125       ─               SOLVE               ─
...         ─               ...
0.250       SOLVE           SOLVE               SOLVE           100 Hz: all 3 run


RATIOS:
  Attitude : Altitude = 40 Hz : 4 Hz = 10:1
  Attitude : Planar = 40 Hz : 4 Hz = 10:1
  Altitude : Planar = 4 Hz : 4 Hz = 1:1 (synchronized)
```

**Design rationale:**
- Attitude runs 10× faster than altitude/planar (angular dynamics are faster)
- Altitude and planar synchronized (both 4 Hz) to avoid conflicts
- If altitude updates altitude reference, planar doesn't have to recompute
- Attitude intermediate results used immediately by downstream layers

---

## Motor Mixing: Thrust/Torque to RPM

```
                 [F_thrust]     [Ω₁ (Motor 1)]
                 [τ_roll  ]  =  [Ω₂ (Motor 2)]
  M⁻¹  such that [τ_pitch ]  M  [Ω₃ (Motor 3)]
                 [τ_yaw   ]     [Ω₄ (Motor 4)]


Motor arrangement (top view):
        Front (pitch axis)
                 │
        1 ╱───┐  │  ┌───╲ 3
         ╱     │  │  │     ╲
    ╱─────────┘  │  └─────────╲
   │             │             │
Roll ─                          ─ Yaw
axis  │             │             │
    ╲─────────┐     ┌─────────╱
         ╲     │  │  │     ╱
        2 ╲───┘  │  └───╱ 4
                 │
              Rear


MOTOR MATRIX (Motor_Mapping_7_Inch.m):
┌──────────────────────────────────────────────────────────┐
│  Motor 1 = 0.25·Fz - (1/2L_roll)·τ_roll                 │
│                  + (1/2L_pitch)·τ_pitch + (K_tau/4)·τ_yaw│
│                                                          │
│  Motor 2 = 0.25·Fz - (1/2L_roll)·τ_roll                 │
│                  - (1/2L_pitch)·τ_pitch - (K_tau/4)·τ_yaw│
│                                                          │
│  Motor 3 = 0.25·Fz + (1/2L_roll)·τ_roll                 │
│                  + (1/2L_pitch)·τ_pitch - (K_tau/4)·τ_yaw│
│                                                          │
│  Motor 4 = 0.25·Fz + (1/2L_roll)·τ_roll                 │
│                  - (1/2L_pitch)·τ_pitch + (K_tau/4)·τ_yaw│
│                                                          │
│  where:                                                  │
│    L_roll = 0.254 m   (254 mm, motor-to-motor)         │
│    L_pitch = 0.2032 m (203.2 mm, motor-to-motor)       │
│    K_tau = 68.9055 (yaw moment scale)                   │
└──────────────────────────────────────────────────────────┘

CONSTRAINTS:
  0 ≤ Motor[i] ≤ 1.0 (normalized, maps to 0–5.11 N per motor)
  If any motor saturates → loss of authority in that axis
```

---

## Initialization and Offline Computation

```
MATLAB Startup Sequence:
└─ Setup_QDrone2_MPC.m
   │
   ├─ Load drone parameters (m, g, J_xx, J_yy, J_zz, motor constants)
   │
   ├─ [ALTITUDE LAYER]
   │  ├─ Discretize continuous model c2d(A_zc, B_zc, Ts=0.25s)
   │  ├─ Cost_Funct() → compute H, F1, F2, F3, F4 (altitude)
   │  └─ InConstraints() → expand thrust bounds across horizon
   │
   ├─ [ATTITUDE LAYER]
   │  ├─ Discretize continuous model c2d(A_ac, B_ac, Ts=0.025s)
   │  ├─ Cost_Funct() → compute H, F1, F2, F3, F4 (attitude)
   │  ├─ InConstraints() → expand torque bounds
   │  └─ Ineq_Calc() → compute Aineq, G1, G2, G3 (angle/rate constraints)
   │
   ├─ [PLANAR LAYER]
   │  ├─ Discretize continuous model c2d(A_lc, B_lc, Ts=0.25s)
   │  ├─ Cost_Funct() → compute H, F1, F2, F3, F4 (planar)
   │  ├─ InConstraints() → expand angle command bounds
   │  └─ Ineq_Calc() → compute Aineq, G1, G2, G3 (position/rate constraints)
   │
   └─ Motor_Mapping_7_Inch.m → load motor mixing matrix M
```

**Result:** All MPC matrices pre-computed and loaded into Simulink workspace.
**Runtime:** Only quadprog(H, F, Aineq, G) executes per cycle.

---

## Real-Time Implemention in Simulink

```
QD2_DroneStack_CompleteMPC_2021a.slx structure:

┌─────────────────────────────────────────────────────────┐
│  Main Simulink Model                                    │
│  (runs on QUARC real-time Linux at 1000 Hz base rate)  │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌──────────────────────────────────────────────────┐  │
│  │ Reference Trajectory (Joystick, Waypoints)      │  │
│  │  • z_cmd (altitude)                             │  │
│  │  • x_cmd, y_cmd (position)                      │  │
│  │  • ψ_cmd (yaw angle)                            │  │
│  └──────────────────────────────────────────────────┘  │
│                                    │                    │
│  ┌────────────────────────────────▼──────────────────┐ │
│  │ MPC Block 1: Altitude (triggered @ 4 Hz)        │ │
│  │  • Input: x_z[k], z_cmd[k]                      │ │
│  │  • Solver: quadprog(H_z, F_z, Aineq_z, G_z)    │ │
│  │  • Output: F_thrust[k]                          │ │
│  └───────┬──────────────────────────────────────────┘ │
│          │                                              │
│  ┌───────▼──────────────────────────────────────────┐ │
│  │ MPC Block 2: Attitude (triggered @ 40 Hz)       │ │
│  │  • Input: x_a[k], φ_cmd[k], θ_cmd[k], ψ_cmd[k]│ │
│  │  • Solver: quadprog(H_a, F_a, Aineq_a, G_a)    │ │
│  │  • Output: τ_roll, τ_pitch, τ_yaw               │ │
│  └───────┬──────────────────────────────────────────┘ │
│          │                                              │
│  ┌───────▼──────────────────────────────────────────┐ │
│  │ MPC Block 3: Planar (triggered @ 4 Hz)          │ │
│  │  • Input: x_l[k], x_cmd[k], y_cmd[k]            │ │
│  │  • Solver: quadprog(H_l, F_l, Aineq_l, G_l)    │ │
│  │  • Output: φ_cmd[k], θ_cmd[k]                   │ │
│  │  • Feeds to Block 2 as setpoints                │ │
│  └───────┬──────────────────────────────────────────┘ │
│          │                                              │
│  ┌───────▼──────────────────────────────────────────┐ │
│  │ Motor Mixing & Saturation                        │ │
│  │  • [Ω₁, Ω₂, Ω₃, Ω₄] = M⁻¹[Fz τx τy τz]        │ │
│  │  • Clamp: 0 ≤ Ω[i] ≤ 1.0 (ESC normalized)      │ │
│  └───────┬──────────────────────────────────────────┘ │
│          │                                              │
│  ┌───────▼──────────────────────────────────────────┐ │
│  │ Sensor Interfaces                                │ │
│  │  • IMU Input: [φ φ̇ θ θ̇ ψ ψ̇]ᵀ → x_a         │ │
│  │  • Barometer: z → x_z                            │ │
│  │  • GPS: [x y ẋ ẏ]ᵀ → x_l                       │ │
│  └───────┬──────────────────────────────────────────┘ │
│          │ (feedback loops)                             │
│          └──────────────────────────────────────────────┘
│
└─────────────────────────────────────────────────────────┘
```

---

## Tuning Parameters and Their Effects

| Parameter | Layer | Effect | Tune If |
|-----------|-------|--------|---------|
| **Qy** (output weight) | All | Higher → tighter tracking, more overshoot | Steady-state error too large |
| **Qu** (input weight) | All | Higher → smoother inputs, slower response | Control jerky or oscillatory |
| **N** (horizon) | All | Longer → better preview, more computation | Overshoots or slow response |
| **Ts** (sample time) | All | Shorter → faster feedback, higher load | Slow transient response |
| **y_max, y_min** | Altitude, Attitude, Planar | Hard state bounds | Physical limits violated |
| **u_max, u_min** | All | Hard input bounds | Saturation effects visible |
| **Delmax, Delmin** | All | Rate-of-change limits | Command slew excessive |

---

## Summary

This hierarchical MPC architecture provides:

1. **Modularity** — Three independent optimization problems (altitude, attitude, planar)
2. **Efficiency** — Tailored horizons and rates per layer (exploit time-scale separation)
3. **Robustness** — Explicit constraint handling prevents saturation instability
4. **Predictive control** — Finite-horizon optimization enables smooth reference tracking
5. **Real-time feasibility** — QP solver on modern hardware (< 5 ms per cycle)

The architecture scales to larger problems (longer horizons, more states) by splitting further or increasing compute resources.
