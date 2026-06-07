function f = Pi_i(i,nx,N)
% PI_I  Selector matrix extracting the i-th block from a stacked vector.
%
% MATHEMATICAL INTENT:
%   Constructs a projection matrix that extracts state/input at timestep i
%   from a stacked horizon-wide vector. Used to decouple tracking cost
%   across timesteps in MPC cost function computation.
%
% PROBLEM CONTEXT:
%   The MPC optimization variables are stacked:
%       x_stacked = [x[0]; x[1]; ...; x[N-1]]  ∈ ℝ^(N·nx)
%       u_stacked = [u[0]; u[1]; ...; u[N-1]]  ∈ ℝ^(N·nu)
%
%   To extract x[i] or u[i] from the stacked form:
%       x[i] = Pi_i(i, nx, N) · x_stacked
%       u[i] = Pi_i(i, nu, N) · u_stacked
%
% COST FUNCTION APPLICATION:
%   Cost at timestep i: J[i] = ‖y[i] - r[i]‖²_Q + ‖u[i]‖²_R
%   where y[i] = C·x[i], r[i] = reference[i]
%
%   Decomposition:
%       Pi_ny = Pi_i(i, nr, N)  → extracts y[i] components
%       Pi_nu = Pi_i(i, nu, N)  → extracts u[i] components
%
%   Then in Cost_Funct:
%       tracking_cost[i] = (Pi_ny)' · Q · Pi_ny
%       input_cost[i] = (Pi_nu)' · R · Pi_nu
%   These are summed across i=1:N to form complete horizon cost.
%
% CONSTRAINT APPLICATION:
%   State bounds: y_min ≤ C·x[i] ≤ y_max for each i
%   Using Pi_i: expands single bound to all N timesteps
%   Input bounds: u_min ≤ u[i] ≤ u_max for each i
%
% INPUTS:
%   i   ∈ {1, 2, ..., N}  Timestep index (1-indexed)
%   nx  ∈ ℤ⁺             State or input dimension
%   N   ∈ ℤ⁺             Prediction horizon
%
% OUTPUT:
%   f ∈ ℝ^(nx × N·nx)     Selection matrix
%
% EXAMPLE:
%   Given x_stacked = [x[0]; x[1]; x[2]; x[3]] ∈ ℝ⁴ (N=4, nx=1)
%   Pi_i(2, 1, 4) = [0 1 0 0] extracts x[1] (second element, 0-indexed)
%
%   More generally for nx=2:
%   Pi_i(2, 2, 3) ∈ ℝ^(2×6) with:
%     Pi_i(2,2,3) = [ 0 0 │ 1 0 │ 0 0 ]  ← selects x[1] (second timestep)
%                   [ 0 0 │ 0 1 │ 0 0 ]
%
% MATHEMATICAL FORM:
%   f[j, (i-1)·nx + k] = δ_{jk}  (Kronecker delta)
%   i.e., identity block at position (i-1)·nx : i·nx
%
% WHY THIS DECOMPOSITION:
%   - Allows per-timestep cost computation without full horizon expansion
%   - Reduces code complexity in Cost_Funct and Ineq_Calc
%   - Explicitly models receding-horizon MPC structure
%
% REFERENCE:
%   Used extensively in Cost_Funct (tracking cost) and Ineq_Calc (bounds)

f = zeros(nx, N*nx);
% Identity block at position [i] in stacked form
% f extracts elements (i-1)·nx+1 to i·nx from input
f(:, (i-1)*nx+1:i*nx) = eye(nx);

return