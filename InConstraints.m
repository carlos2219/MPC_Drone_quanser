function [max_u,min_u] = InConstraints(u_max,u_min,N)
% INCONSTRAINTS  Replicates single-step input bounds across prediction horizon.
%
% MATHEMATICAL INTENT:
%   Expands per-step input constraints to a horizon-wide constraint vector
%   for the MPC finite-horizon optimization. This is a simple utility that
%   enforces identical input bounds at every future timestep.
%
% INPUT CONSTRAINTS IN MPC:
%   The MPC QP has decision variable u ∈ ℝ^(N·nu):
%       u = [u[0]; u[1]; ...; u[N-1]]  (stacked control sequence)
%
%   Per-timestep bound: u_min ≤ u[k] ≤ u_max for k=0,...,N-1
%
%   This is enforced as affine constraint: u_min_expanded ≤ u ≤ u_max_expanded
%   where:
%       u_min_expanded = [u_min; u_min; ...; u_min]  ∈ ℝ^(N·nu)
%       u_max_expanded = [u_max; u_max; ...; u_max]  ∈ ℝ^(N·nu)
%
% PHYSICAL MOTIVATION:
%   Motor constraints (thrust, torque limits) are same at each sample
%   → u_max/u_min do not change with horizon index
%   → Simple replication (vs. time-varying bounds from varying references)
%
% INPUTS:
%   u_max  ∈ ℝ^nu         Upper bound on control input (per step)
%   u_min  ∈ ℝ^nu         Lower bound on control input (per step)
%   N      ∈ ℤ⁺           Prediction horizon (number of steps)
%
% OUTPUTS:
%   max_u  ∈ ℝ^(N·nu)     Horizon-wide upper bound vector
%                         max_u = [u_max; u_max; ...; u_max]  (N copies)
%   min_u  ∈ ℝ^(N·nu)     Horizon-wide lower bound vector
%                         min_u = [u_min; u_min; ...; u_min]  (N copies)
%
% EXAMPLES:
%   Altitude control (nu=1, N=10):
%     Input: u_max = 5.11 N (thrust), u_min = 0 N, N = 10
%     Output: max_u = [5.11; 5.11; ...; 5.11]  ∈ ℝ^10
%             min_u = [0;    0;    ...; 0]      ∈ ℝ^10
%
%   Attitude control (nu=3, N=15):
%     Input: u_max = [1.64; 1.35; 0.19] rad (roll/pitch/yaw), N=15
%     Output: max_u ∈ ℝ^45 (15 copies of 3 bounds each)
%             min_u ∈ ℝ^45 (15 copies of 3 negative bounds)
%
% WHY NOT BUILD THIS INTO QP SOLVER:
%   - MATLAB's quadprog expects explicit constraint vector (not index range)
%   - Horizon-wide vector fits standard QP form: Aineq·u ≤ G
%   - Simple replication avoids complicated indexing logic
%
% USAGE IN MPC:
%   [max_u, min_u] = InConstraints(u_max, u_min, N)
%   Then in quadprog: options.UB = max_u, options.LB = min_u
%   Or manually: Aineq·u ≤ G where G includes ±u_min_expanded, ±u_max_expanded
%
% ALTERNATIVE FORMULATION:
%   Could use repmat: max_u = repmat(u_max, N, 1)
%   But loop form is explicit and matches MATLAB MPC literature style

max_u = [];
min_u = [];

% Stack N copies of per-step bounds
for i = 1:N
    max_u = [max_u; u_max];  % Append u_max to column vector
    min_u = [min_u; u_min];  % Append u_min to column vector
end