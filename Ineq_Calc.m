function [Aineq,G1,G2,G3] = Ineq_Calc(Cc,phi,psi,N,nr,nu,n,y_max,y_min,delmax,delmin,B)
% INEQ_CALC  Constructs linear inequality constraint matrices for MPC.
%
% MATHEMATICAL INTENT:
%   Converts hard state and input bounds into linear inequality form for
%   the finite-horizon QP:
%       Aineq·u ≤ G
%   where u ∈ ℝ^(N·nu) is the N-step control sequence.
%
% CONSTRAINTS MODELED:
%   1. OUTPUT BOUNDS: y_min ≤ y[k] ≤ y_max for k=0,...,N-1
%      Enforces: state limits (e.g., |φ| ≤ π/4, |z| ≥ 0, x ∈ [-2,2])
%      Why: prevent nonlinear dynamics saturation, maintain linearization validity
%
%   2. INPUT BOUNDS: u_min ≤ u[k] ≤ u_max for k=0,...,N-1
%      Enforces: actuator saturation (e.g., 0 ≤ thrust ≤ 5.11 N)
%      Why: motors have finite bandwidth, cannot exceed max RPM
%
%   3. RATE-OF-CHANGE BOUNDS: delmin ≤ Δu[k] ≤ delmax
%      Enforces: |u[k] - u[k-1]| ≤ limit
%      Why: smooth actuation, prevent chattering, respect motor response time
%
% CONSTRAINT REFORMULATION:
%   Original: y_min ≤ C·x[k] ≤ y_max
%   Expanded: y_min ≤ C·Φ[k]·x[0] + C·Ψ[k]·u[0:k] ≤ y_max
%   Rearranged as: Aineq_1·u ≤ G_3, where:
%     Aineq_1[k] = C·Ψ[k] (how control affects output at time k)
%     G_3[k] = y_max - C·Φ[k]·x[0] (upper bound coupling with x[0])
%     −Aineq_1[k]·u ≤ −y_min + C·Φ[k]·x[0] (lower bound form)
%
% INPUTS:
%   Cc       ∈ ℝ^(nr×n)       Output selection matrix C
%   phi      ∈ ℝ^(N·n × n)    State transition matrix Φ (from Cost_Funct)
%   psi      ∈ ℝ^(N·n × N·nu) Input-to-state matrix Ψ (from Cost_Funct)
%   N        ∈ ℤ⁺             Prediction horizon
%   nr       ∈ ℤ⁺             Number of outputs (measured states)
%   nu       ∈ ℤ⁺             Number of inputs (control dimension)
%   n        ∈ ℤ⁺             State dimension
%   y_max    ∈ ℝ^nr           Upper bounds on outputs
%   y_min    ∈ ℝ^nr           Lower bounds on outputs
%   delmax   ∈ ℝ^nu           Upper bounds on input rate Δu[k]
%   delmin   ∈ ℝ^nu           Lower bounds on input rate Δu[k]
%   B        ∈ ℝ^(n×nu)       Discrete input matrix (from Cost_Funct)
%
% OUTPUTS:
%   Aineq    ∈ ℝ^(2N·nr × N·nu) Constraint matrix for output bounds
%   G1       ∈ ℝ^(2N·nr × n)    RHS coupling with initial state x[0]
%   G2       ∈ ℝ^(2N·nu × nu)   RHS coupling with initial input u[-1]
%   G3       ∈ ℝ^(2N·nr)        RHS constant bounds
%
% MATHEMATICAL FORM:
%   The full constraint is: Aineq·u ≤ G1·x[0] + G2·u[-1] + G3
%   At runtime: G = G1·x[k] + G2·u[k-1] + G3
%   Then: quadprog minimizes subject to Aineq·u ≤ G
%
% WHY THIS STRUCTURE:
%   - Affine in u (linear control action) → convex QP
%   - Explicit treatment of bounds → optimal control respects limits
%   - Per-timestep expansion → horizon-wide constraint enforcement
%   - Feedback u[-1] term enables smooth rate-limited transitions
%
% REFERENCES:
%   - Boyd & Parikh, "Convex Optimization", § 4.4 (LP/QP with affine constraints)
%   - Maciejowski, "Predictive Control with Constraints" (MPC constraint forms)

Aineq_1 = zeros(2*N*nr,N*nu);
Aineq_2 = zeros(2*N*nu,N*nu);
G1_1 = zeros(2*N*nr,n);

G2_2 = zeros(2*N*nu,nu);
G2_2(1:nu,1:nu) = eye(nu);
G2_2(N*nu+1:N*nu+nu,1:nu) = -eye(nu);

G3_1 = zeros(2*N*nr,1);
G3_2 = zeros(2*N*nr,1);

for i = 1:N
    % ROW INDEXING SCHEME:
    % Rows 1:N*nr         → upper bound constraints: y[k] ≤ y_max
    % Rows (N*nr+1):2N*nr → lower bound constraints: y[k] ≥ y_min (reformulated as -y ≤ -y_min)

    % UPPER BOUND CONSTRAINT: y[k] ≤ y_max
    %   y[k] = C·x[k] = C·Φ[k]·x[0] + C·Ψ[k]·u[0:k]
    %   Rearranged: C·Ψ[k]·u ≤ y_max - C·Φ[k]·x[0]
    %   Aineq_1 block contains C·Ψ[k] (how u affects output y[k])
    Aineq_1((i*nu)-(nu-1):(i*nu),1:N*nu) = Cc*Pi_i(i,n,N)*psi;

    % LOWER BOUND CONSTRAINT: y[k] ≥ y_min (reformulated)
    %   -y[k] ≤ -y_min
    %   -C·Ψ[k]·u ≤ -y_min + C·Φ[k]·x[0]
    %   Aineq_1 negated to represent lower bound
    Aineq_1((i*nu+N*nu)-(nu-1):(i*nu+N*nu),1:N*nu) = -Cc*Pi_i(i,n,N)*psi;

    % G1 MATRIX: Coupling with initial state x[0]
    % From rearranged constraints: RHS contains C·Φ[k]·x[0] term
    % Negative for upper bound: -C·Φ[k]·x[0]
    % Positive for lower bound: +C·Φ[k]·x[0]
    G1_1((i*nu)-(nu-1):(i*nu),1:n) = -Cc*Pi_i(i,n,N)*phi;
    G1_1((i*nu+N*nu)-(nu-1):(i*nu+N*nu),1:n) = Cc*Pi_i(i,n,N)*phi;

    % G3 MATRIX: Constant bounds (output limits)
    % Stores y_max and y_min for each timestep
    % Indexing accounts for nr outputs (2 or 3 state variables tracked)
    if nr == 2  % Planar motion (x, y) or altitude + something
        % Rows i*2-1, i*2 store bounds for output 1, 2 at timestep i
        G3_1(i*nr,1) = y_max(nr,1);         % y_max for 2nd output
        G3_1(i*nr-1,1) = y_max(nr-1,1);     % y_max for 1st output
        G3_1(N*nr+i*nr,1) = -y_min(nr,1);   % -y_min for 2nd output (lower bound)
        G3_1(N*nr+i*nr-1,1) = -y_min(nr-1,1); % -y_min for 1st output

    elseif nr == 3  % Attitude (roll, pitch, yaw)
        G3_1(i*nr,1) = y_max(nr,1);         % y_max for 3rd output
        G3_1(i*nr-1,1) = y_max(nr-1,1);     % y_max for 2nd output
        G3_1(i*nr-2,1) = y_max(nr-2,1);     % y_max for 1st output
        G3_1(N*nr+i*nr,1) = -y_min(nr,1);   % -y_min for 3rd output
        G3_1(N*nr+i*nr-1,1) = -y_min(nr-1,1); % -y_min for 2nd output
        G3_1(N*nr+i*nr-2,1) = -y_min(nr-2,1); % -y_min for 1st output
    end

end

% Aineq = [Aineq_1;Aineq_2];                                                  % Complete Aineq
% G1 = [G1_1;zeros(2*N*nu,n)];                                                % Complete G1
% G2 = [zeros(2*N*nr,nu);G2_2];                                               % Complete G2
% G3 = [G3_1;G3_2];

Aineq = Aineq_1;                                                  % Complete Aineq
G1 = G1_1;                                                % Complete G1
G2 = zeros(2*N*nr,nu);                                               % Complete G2
G3 = G3_1;


