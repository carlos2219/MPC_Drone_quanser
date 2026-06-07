% SETUP_QDRONE2_MPC  Initialize hierarchical MPC control system for QDrone2.
%
% OVERVIEW:
%   This script initializes all parameters, discretizes continuous-time
%   dynamics, and pre-computes MPC cost and constraint matrices for the
%   three-layer hierarchical control architecture:
%     - Altitude MPC     (4 Hz, N=10): vertical position tracking
%     - Attitude MPC    (40 Hz, N=15): roll/pitch/yaw stabilization
%     - Planar MPC       (4 Hz, N=15): horizontal trajectory tracking
%
% EXECUTION FLOW:
%   1. Define drone physical parameters (mass, inertia, motor constants)
%   2. For each MPC layer:
%      a. Specify LTI dynamics (A, B, C matrices)
%      b. Discretize continuous model via c2d() at layer's sample time
%      c. Define output/input weights (Qy, Qu)
%      d. Compute Hessian & linear cost coefficients (Cost_Funct)
%      e. Compute constraint matrices (Ineq_Calc)
%      f. Load bounds (state limits, input saturation, rate limits)
%   3. Compute motor mixing matrix (inverse kinematics)
%   4. All results loaded into Simulink workspace for real-time control
%
% KEY DESIGN DECISIONS:
%   - Altitude @ 4 Hz: slower vertical dynamics allow long horizon (N=10)
%   - Attitude @ 40 Hz: fast angular dynamics, short horizon (N=15)
%   - Planar @ 4 Hz: synchronized with altitude for coherent control
%   - Decoupled layers: each solves small QP independently
%   - Disturbance term: gravity explicitly modeled in altitude layer
%
% MATHEMATICAL CHAIN:
%   Continuous LTI: ẋ = Ax + Bu + Gd
%   Discretized:    x[k+1] = Ax[k] + Bu[k] + Gd[k]
%   MPC finite horizon: min Σ J[k] subject to state/input bounds
%   Quadratic form: min ½u'Hu + F'u subject to Aineq·u ≤ G
%   Runtime solver: quadprog(H, F, Aineq, G) every Ts seconds
%
% OUTPUT VARIABLES (loaded to Simulink workspace):
%   H_z, F1_z, F2_z, F3_z, F4_z, phi_z, psi_z  (altitude layer)
%   H_a, F1_a, F2_a, F3_a, F4_a, phi_a, psi_a  (attitude layer)
%   Aineq_a, G1_a, G2_a, G3_a                   (attitude constraints)
%   H_l, F1_l, F2_l, F3_l, F4_l, phi_l, psi_l  (planar layer)
%   Aineq_l, G1_l, G2_l, G3_l                   (planar constraints)
%   Motor_Matrix (and implicitly Motor_Matrix_Inv)
%   max_ua, min_ua, max_ul, min_ul              (input bounds)
%
% TUNING PARAMETERS:
%   Change Qy_* and Qu_* to adjust tracking aggressiveness
%   Change N_* and ts_* to modify horizon/sampling rate
%   Change ya_max/ya_min, yl_max/yl_min for constraint tightness
%   Change delmax/delmin for rate-limiting effect
%
% DEPENDENCIES:
%   MATLAB Control System Toolbox: c2d (discretization)
%   Optimization Toolbox: quadprog (solver, called at runtime)
%   Custom functions: Cost_Funct, Ineq_Calc, InConstraints, Pi_i
%
% REFERENCES:
%   System identification: Quanser QDrone2 Technical Manual
%   MPC formulation: Rawlings & Mayne, "Model Predictive Control"
%   Implementation: Boyd & Parikh, "Convex Optimization"
%
% NOTE:
%   This script should be run once at startup to populate Simulink
%   workspace. Do NOT re-run during flight (would reset optimizer state).
%   To update parameters, modify values below, re-run, then re-initialize
%   Simulink model.

%clearvars
%clc

%% DRONE PHYSICAL PARAMETERS
% Taken from Quanser QDrone2 specification sheet
% Units: SI (kg, m, rad, N, Nm)

m = 1.504;       %Drone total mass [kg]   2 
g = 9.81;        %Gravity force [m/s^2]
lr = 0.2136;     %Roll motor-to-motor distance [m]
lp = 0.1758;     %Pitch motor-to-motor distance [m]
kf = 5.11;       %Max commanded thrust force [N]
kt = 0.0487;     %Yaw torque constant [Nm]
k = 81.0363;     %Motor thrust-torque constant [N/Nm]
fb = -0.2046;    %Motor force offset [N]
ct = 2.0784e-8;  %Motor force constant [N/RPM^2]
wf = 1004.5;     %Angular velocity to force offset [RPM]
wc = 2132.6;     %Voltage to angular velocity offset [RPM]
kv = 1295.4;     %Effective motor speed constant [RPM/V]
Lside   = 0.2136/2;     % Sideway offset (m)
Laft    = 0.1758/2;     % Forward and aft offset (m)
mRotor  = 0.045;    % Each, including motor and prop mass (kg)
Kv = 2100*2*pi/60;  % 2100 RPM/V => 70 pi rad/s/V
Kt = 1/Kv;          % N/A
Imax = 5.82;        % Amp
Tmax = 5.11;        % N
Tm = 0.04;       % seconds
tau = 0.02; %I used this in my model...
Cm_p = 16.2115;       % kRPM
omega_b_p = 1.50575;  % kRPM
Ct_p = 0.01871;       % N/kRPM^2
Ctau_p = 0.00024669;  % Nm/kRPM^2
Cm = 1.2954;       % kRPM
omega_b = 2.1326;    % kRPM
Ct = 0.020784;       % N/kRPM^2
Ctau = 0.00025648;  % Nm/kRPM^2 (Ct/k_tau)

% QDrone 1
% Jxx = 0.010035;        % kg*m^2
% Jyy = 0.008225;        % kg*m^2
% Jzz = 0.014783;        % kg*m^2

% QDrone 2
Jxx = 0.01277;        % kg*m^2
Jyy = 0.01337;        % kg*m^2
Jzz = 0.03047;        % kg*m^2


%% ALTITUDE CONTROL MPC LAYER
%
% DESIGN RATIONALE:
%   Vertical dynamics are decoupled from roll/pitch (small-angle regime)
%   and slower (natural frequency ~1-2 Hz vs. ~5 Hz for attitude)
%   → Allows longer prediction horizon (N=10) and slower sampling (0.25s)
%   → Reduces computation while maintaining smooth altitude tracking
%
% STATE VECTOR: x_z = [z, ż]ᵀ
%   z: altitude (m), measured from barometer/GPS
%   ż: vertical velocity (m/s), estimated from barometer derivative
%
% CONTROL INPUT: u_z = F_z
%   F_z: total vertical thrust command to all motors (N)
%   Range: [0, 5.11] N (4 motors × 1.2775 N max per motor, but typically limited)
%
% MEASURED OUTPUT: y_z = z (altitude only, no velocity feedback in cost)
%
% COST FUNCTION WEIGHTS:
%   Qy_z = 10:      High altitude tracking priority
%           → strongly penalizes altitude error (reference - measured)
%           → aggressive setpoint changes tracked quickly
%   Qu_z = 0.00001: Very soft input penalization
%           → allows large thrust commands without penalty
%           → MPC will saturate thrust if needed for tracking
%   (No rate-of-change penalty S_z, so thrust can jump between setpoints)

N_z = 10;        % Prediction horizon: 10 steps × 0.25s = 2.5s preview
ts_z = 0.25;     % Sampling time: 4 Hz (slow vertical dynamics)

Qy_z = 10;       % Altitude tracking weight (diagonal)
Qu_z = 0.00001;  % Thrust effort weight (scalar, very small)

u_dz = 0;        % Dead zone (not used; set to zero)

% CONTINUOUS-TIME MODEL: ż̇ = F_z/m - g
%   State vector form: [ż̇] = [0 1] [z] + [0  ] F_z + [0 ] (-g)
%                      [z̈]   [0 0] [ż]   [1/m]       [-g]
%   (Mass on bottom, gravity as constant disturbance)

A_zc = [0 1; 0 0];          % Continuous state transition
B_zc = [0; 1/m];            % Continuous input matrix (1/m scaling from F=ma)
C_z = [1 0];                % Output: measure altitude only
G = [0; -g];                % Disturbance: gravity (affects acceleration, not position)

% DISCRETIZATION: c2d with zero-order hold at Ts=0.25s
%   Converts continuous ẋ = Ax + Bu + Gd to x[k+1] = Ad·x[k] + Bd·u[k] + Gd·d[k]
%   Uses matrix exponential: Ad = exp(A·Ts), Bd = ∫₀^Ts exp(A·τ)dτ B
[A_z, B_z] = c2d(A_zc, B_zc, ts_z); 

[n_z,nu_z] = size(B_z);
nr_z = size(C_z,1);

[H_z,F1_z,F2_z,F3_z,F4_z,phi_z,psi_z,GMat] = Cost_Funct(A_z,B_z,C_z,G,Qy_z,Qu_z,N_z,1);

F4_prueb = [];
for i = 1:N_z
    F4_prueb(i,1) = F4_z(N_z+1-i);
end

%% ATTITUDE CONTROL MPC LAYER
%
% DESIGN RATIONALE:
%   Angular dynamics are fastest (gyroscope bandwidth ~50+ Hz)
%   → Requires shortest prediction horizon (N=15) and fastest sampling (0.025s = 40 Hz)
%   → Stabilizes roll/pitch/yaw while receiving setpoints from planar & altitude layers
%   → Decoupled control axes (no cross-coupling terms in linear model)
%
% STATE VECTOR: x_a = [φ, φ̇, θ, θ̇, ψ, ψ̇]ᵀ
%   φ, θ, ψ: Euler angles (roll, pitch, yaw) in radians
%   φ̇, θ̇, ψ̇: Angular rates (rad/s), measured by IMU gyroscope
%
% CONTROL INPUTS: u_a = [τ_roll, τ_pitch, τ_yaw]ᵀ
%   Torques about body-frame axes (Nm)
%   Computed from motor mixing matrix inverse (4 motors → 3 torques + 1 thrust)
%
% MEASURED OUTPUTS: y_a = [φ, θ, ψ]ᵀ (angles, no rate feedback in cost)
%
% COST FUNCTION WEIGHTS:
%   Qy_a = diag([2500, 1500, 1500]):
%     → Roll tracking weight 2500 (more aggressive stabilization)
%     → Pitch tracking weight 1500 (slightly less aggressive)
%     → Yaw tracking weight 1500 (independent yaw control)
%     → Higher weights → stricter angle tracking, faster response
%   Qu_a = diag([250, 550, 500]):
%     → Roll torque penalty 250 (allows ~1.6 N·m commands)
%     → Pitch torque penalty 550 (allows ~1.3 N·m commands)
%     → Yaw torque penalty 500 (allows ~0.2 N·m commands, smallest)
%     → Asymmetric penalties reflect asymmetric motor geometry

N_a = 15;        % Prediction horizon: 15 steps × 0.025s = 0.375s preview
ts_a = 0.025;    % Sampling time: 40 Hz (fast angular dynamics)

% Current tuned weights (higher values = more aggressive)
Qy_a = [2500 0 0; 0 1500 0; 0 0 1500];  % Diagonal output tracking weights
Qu_a = [250 0 0; 0 550 0; 0 0 500];     % Diagonal input effort weights

% Alternative (less aggressive) values commented below:
% Qy_a = [1500 0 0; 0 750 0; 0 0 1500];  % More conservative gains
% Qu_a = [200 0 0; 0 500 0; 0 0 250];    % Higher input penalties

ud_a = [0; 0; 0];  % Dead zone (not used)

% CONTINUOUS-TIME MODEL (Decoupled rigid-body kinematics):
%   φ̈ = τ_roll / Jxx
%   θ̈ = τ_pitch / Jyy
%   ψ̈ = τ_yaw / Jzz
%
% State-space form:
%   ẋ = [φ̇] = [0 1 0 0 0 0] [φ ] + [0   0    0  ] [τ_roll ]
%       [φ̈]   [0 0 0 0 0 0] [φ̇]   [1/J 0    0  ] [τ_pitch]
%       [θ̇]   [0 0 0 1 0 0] [θ ]   [xx  0    0  ] [τ_yaw ]
%       [θ̈]   [0 0 0 0 0 0] [θ̇]   [0   1/Jyy 0 ]
%       [ψ̇]   [0 0 0 0 0 1] [ψ ]   [0   0    0  ]
%       [ψ̈]   [0 0 0 0 0 0] [ψ̇]   [0   0   1/Jzz]

A_ac = [0 1 0 0 0 0;   % Φ̇ = φ̇
        0 0 0 0 0 0;   % Φ̈ = τ_roll/J (input-driven)
        0 0 0 1 0 0;   % θ̇ = θ̇
        0 0 0 0 0 0;   % θ̈ = τ_pitch/J (input-driven)
        0 0 0 0 0 1;   % ψ̇ = ψ̇
        0 0 0 0 0 0];  % ψ̈ = τ_yaw/J (input-driven)

B_ac = [0 0 0;         % No direct input to angle (only to rate)
        1/Jxx 0 0;     % Roll equation
        0 0 0;
        0 1/Jyy 0;     % Pitch equation
        0 0 0;
        0 0 1/Jzz];    % Yaw equation

C_a = [1 0 0 0 0 0;    % Measure roll angle
       0 0 1 0 0 0;    % Measure pitch angle
       0 0 0 0 1 0];   % Measure yaw angle (no rate feedback)

D_a = [0 0 0; 0 0 0; 0 0 0];  % No direct feedthrough

% DISCRETIZATION at Ts = 0.025s (40 Hz)
[A_a, B_a] = c2d(A_ac, B_ac, ts_a); 
angss = ss(A_a,B_a,C_a,D_a,0.01);

% A_a = A_ac;
% B_a = B_ac;

[n_a,nu_a] = size(B_a);
nr_a = size(C_a,1);

ua_max = [1.6373;1.3476;0.1892];  
ua_min = [-1.6373;-1.3476;-0.1892];

ya_max = [pi/4;pi/4;100*pi/180];
ya_min = [-pi/4;-pi/4;-100*pi/180];

% ya_max = [1.6373;1.3476;0.1892];
% ya_min = [-1.6373;-1.3476;-0.1892];

delmax_a = [0.1963;0.1963;pi/8];
delmin_a = [-0.1963;-0.1963;-pi/8];

[max_ua,min_ua] = InConstraints(ua_max,ua_min,N_a);
[H_a,F1_a,F2_a,F3_a,F4_a,phi_a,psi_a] = Cost_Funct(A_a,B_a,C_a,G,Qy_a,Qu_a,N_a,2);
[Aineq_a,G1_a,G2_a,G3_a] = Ineq_Calc(C_a,phi_a,psi_a,N_a,nr_a,nu_a,n_a,ya_max,ya_min,delmax_a,delmin_a,B_a);


%% PLANAR MOTION CONTROL MPC LAYER
%
% DESIGN RATIONALE:
%   Horizontal (x-y) dynamics are slower than attitude (~1-2 Hz natural freq)
%   Synchronized with altitude at 4 Hz for coordinated control
%   Outputs roll/pitch angle commands to attitude MPC (setpoints)
%   Decoupled x and y axes (quadrotor symmetry)
%
% STATE VECTOR: x_l = [x, ẋ, y, ẏ]ᵀ
%   x, y: horizontal position (m), measured by GPS
%   ẋ, ẏ: horizontal velocities (m/s), estimated from GPS derivative
%
% CONTROL INPUTS: u_l = [φ_cmd, θ_cmd]ᵀ
%   Roll and pitch angle COMMANDS to attitude layer (radians)
%   These become setpoints for attitude MPC (y_a_ref = [φ_cmd, θ_cmd, ψ_cmd])
%   Constraints: |φ_cmd|, |θ_cmd| ≤ π/4 rad (±45°)
%
% MEASURED OUTPUTS: y_l = [x, y]ᵀ (positions, no velocity feedback in cost)
%
% COST FUNCTION WEIGHTS:
%   Qy_l = diag([100, 500]):
%     → X-position weight 100 (moderate tracking priority)
%     → Y-position weight 500 (higher priority, 5× stronger)
%     → Asymmetric: Y-axis motion more tightly controlled
%   Qu_l = diag([300, 800]):
%     → Roll command penalty 300 (allows ~0.2 rad commands)
%     → Pitch command penalty 800 (allows ~0.16 rad commands)
%     → Higher penalties → smoother, less jerky angle commands
%
% CONTROL COUPLING:
%   Small-angle approximation (valid for |φ|, |θ| < π/4):
%     ẍ ≈ g·φ        (horizontal accel from roll tilt)
%     ÿ ≈ -g·θ       (horizontal accel from pitch tilt, opposite sign)
%   This is gravity-driven coupling; strong constraint enforcement prevents
%   excessive tilts that would violate linearization assumption.

N_l = 15;        % Prediction horizon: 15 steps × 0.25s = 3.75s preview
ts_l = 0.25;     % Sampling time: 4 Hz (synchronized with altitude)

% Current tuned weights
Qy_l = [100 0; 0 500];       % X: 100, Y: 500 (y-emphasis)
Qu_l = [300 0; 0 800];       % Roll penalty: 300, Pitch penalty: 800

% Alternative (more balanced) tuning commented:
% Qy_l = [50 0; 0 50];        % Balanced, looser tracking
% Qu_l = [1 0; 0 1];          % Very soft angle commands

ud_l = [0; 0];  % Dead zone (not used)

% CONTINUOUS-TIME MODEL (Decoupled, small-angle, gravity-driven):
%   ẍ = g·sin(φ) ≈ g·φ      (x-axis acceleration from roll)
%   ÿ = -g·sin(θ) ≈ -g·θ    (y-axis acceleration from pitch, opposite sign)
%   Control inputs: φ_cmd, θ_cmd (angle commands are direct control)
%
% State-space form:
%   ẋ = [0 1 0 0] [x ] + [0 0] [φ_cmd]
%       [0 0 g 0] [ẋ]   [0 0] [θ_cmd]
%       [0 0 0 1] [y ]   [0 0]
%       [0 0 0 0] [ẏ]   [-g 0]

A_lc = [0 1 0 0;   % ẋ = ẋ
        0 0 g 0;   % ẍ = g·φ (note: input below)
        0 0 0 1;   % ẏ = ẏ
        0 0 0 0];  % ÿ = -g·θ (note: input below)

B_lc = [0 0;       % No direct input to position
        g 0;       % ẍ coefficient: g per radian roll → φ_cmd couples to ẍ
        0 0;       % No direct input to y velocity
        0 -g];     % ÿ coefficient: -g per radian pitch → θ_cmd couples to ÿ

C_l = [1 0 0 0;    % Measure x position
       0 0 1 0];   % Measure y position (no velocity feedback)

D_l = [0 0; 0 0];  % No direct feedthrough

% DISCRETIZATION at Ts = 0.25s (4 Hz, same as altitude for synchronization)
[A_l, B_l] = c2d(A_lc, B_lc, ts_l); 

[n_l,nu_l] = size(B_l);
nr_l = size(C_l,1);

ul_max = [pi/4;pi/4];    %Setup the upper and lower input constraints (RP commands)
ul_min = [-pi/4;-pi/4];

yl_max = [2;2];
yl_min = [-2;-2];
% yl_max = [pi/8;pi/8];
% yl_min = [-pi/8;-pi/8];

delmax_l = [0.2;0.2];
delmin_l = [-0.2;-0.2];

[max_ul,min_ul] = InConstraints(ul_max,ul_min,N_l);
[H_l,F1_l,F2_l,F3_l,F4_l,phi_l,psi_l] = Cost_Funct(A_l,B_l,C_l,G,Qy_l,Qu_l,N_l,2);
psi_last = psi_l(N_l*n_l-(n_l-1):N_l*n_l,:);
[Aineq_l,G1_l,G2_l,G3_l] = Ineq_Calc(C_l,phi_l,psi_l ,N_l,nr_l,nu_l,n_l,yl_max,yl_min,delmax_l,delmin_l,B_l);

%% Motor Mapping
Motor_Mapping_7_Inch;