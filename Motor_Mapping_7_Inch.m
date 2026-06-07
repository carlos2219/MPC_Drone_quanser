%% =====================Motor Mapping script============================
% MOTOR_MAPPING_7_INCH  Inverse kinematics: thrust/torque → motor RPM
%
% MATHEMATICAL INTENT:
%   Constructs the inverse mixing matrix M⁻¹ that converts desired total
%   thrust and body-frame torques (MPC outputs) into individual motor
%   speed commands. This realizes the nonlinear thrust vector-torque
%   coupling of a quadrotor platform.
%
% QUADROTOR PHYSICS (Small-Angle Approximation):
%   The four motors produce thrust F_i (vertical force) and contribute
%   to roll/pitch/yaw torques based on their lever arms and thrust:
%
%   Total vertical thrust:
%       F_total = F_1 + F_2 + F_3 + F_4
%
%   Roll torque (about x-axis, moment arm L_roll):
%       τ_roll = (-F_1 - F_2 + F_3 + F_4) · L_roll / 2
%       (Motors 1,2 on left produce negative roll; 3,4 on right positive)
%
%   Pitch torque (about y-axis, moment arm L_pitch):
%       τ_pitch = (-F_1 + F_2 - F_3 + F_4) · L_pitch / 2
%       (Motors 1,3 rear produce negative pitch; 2,4 front positive)
%
%   Yaw torque (about z-axis, proportional to motor speed difference):
%       τ_yaw = K_tau · (F_1 - F_2 + F_3 - F_4)
%       (Motor thrust creates aerodynamic drag torque proportional to thrust)
%
% MOTOR ARRANGEMENT (top view):
%               Front (pitch axis)
%                       │
%            1 ╱───┐    │    ┌───╲ 3
%             ╱     │    │    │     ╲
%        ╱─────────┘     │    └─────────╲
%       │                │                │
%   Roll ─                                ─ Yaw
%   axis  │                │                │
%      ╲─────────┐     │    ┌─────────╱
%           ╲     │    │    │     ╱
%            2 ╲───┘    │    └───╱ 4
%                       │
%                    Rear
%
% MIXING MATRIX FORM:
%   [ F_total ]     [ 1    1    1    1  ]   [ F_1 ]
%   [ τ_roll  ]  =  [-a  -a   a   a  ] · [ F_2 ]
%   [ τ_pitch ]     [-b   b  -b   b  ]   [ F_3 ]
%   [ τ_yaw   ]     [ c  -c   c  -c  ]   [ F_4 ]
%
%   where a = L_roll/2, b = L_pitch/2, c = K_tau/4
%
% INVERSE MIXING:
%   The MPC outputs [F_total, τ_roll, τ_pitch, τ_yaw]'
%   We need [F_1, F_2, F_3, F_4]' for motor commands
%   So we compute M⁻¹ and apply: [F_1 F_2 F_3 F_4]' = M⁻¹ · [thrust, torques]'
%
% GEOMETRIC PARAMETERS (7-inch Quanser QDrone):
%   - Propeller diameter: 7 inches = 177.8 mm
%   - Motor-to-motor distance (roll axis): 254 mm
%   - Motor-to-motor distance (pitch axis): 203.2 mm
%   - Yaw inertia much larger than roll/pitch → K_tau < {L_roll, L_pitch}

% Parameters for 7 inch QDrone 

% Geometric parameters (7-inch props)
L_Roll  = 10*(25.4/1000);   % Motor-to-motor distance [m] = 10 inches × 25.4 mm/inch
L_Pitch = 8*(25.4/1000);    % Motor-to-motor distance [m] = 8 inches
K_Tau   = 68.9055;          % Yaw moment scaling factor (empirically tuned)
KT = [0.03616, 0.117, -0.01215];  % Motor control law gains (advanced use)

% MOTOR MIXING MATRIX: Forward kinematics
% Each row represents one force/torque output
% Each column represents one motor contribution
%
% Row 1 (Total Thrust):     all motors contribute equally (0.25 each for normalization)
% Row 2 (Roll Torque):      Motors 1,2 push down (negative), 3,4 push up (positive)
% Row 3 (Pitch Torque):     Motors 1,3 rear, 2,4 front (decoupled axes)
% Row 4 (Yaw Torque):       Alternating signs (2 motors push one direction)
%
% Coefficients:
%   0.25 = 1/4 (four motors share total thrust)
%   1/(2*L_Roll) = moment arm for roll
%   1/(2*L_Pitch) = moment arm for pitch
%   K_Tau/4 = yaw moment per motor
%
% Forward relation: [F_total; τ_roll; τ_pitch; τ_yaw] = Motor_Matrix · [Ω₁; Ω₂; Ω₃; Ω₄]
%
% Motor numbering:
%   1 = front-left,  2 = rear-left
%   3 = front-right, 4 = rear-right

Motor_Matrix = [ 0.25   -1/(2*L_Roll)    1/(2*L_Pitch)    K_Tau/4;...   % Motor 1 contribution
                 0.25   -1/(2*L_Roll)   -1/(2*L_Pitch)   -K_Tau/4;...   % Motor 2 contribution
                 0.25    1/(2*L_Roll)    1/(2*L_Pitch)   -K_Tau/4;...   % Motor 3 contribution
                 0.25    1/(2*L_Roll)   -1/(2*L_Pitch)    K_Tau/4];     % Motor 4 contribution

% INVERSE OPERATION (used in Simulink):
% Motor_Matrix_Inv = inv(Motor_Matrix) applied to MPC-computed [F, τx, τy, τz]
% Output: Individual motor force commands [F₁, F₂, F₃, F₄]
% Constraints: 0 ≤ F_i ≤ T_max (saturation enforced in ESC firmware)