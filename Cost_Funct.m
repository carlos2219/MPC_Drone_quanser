function [H,F1,F2,F3,F4,phi,psi,GMat] = Cost_Funct(A,B,C,G,Qy,Qu,N,c)
% COST_FUNCT  Constructs quadratic cost function matrices for MPC.
%
% MATHEMATICAL INTENT:
%   Converts the discrete-time linear dynamics and tracking cost into a
%   finite-horizon quadratic program (QP) in standard form:
%       min ½u'Hu + F'u
%   subject to: Aineq·u ≤ G (handled separately by Ineq_Calc)
%
% The cost function is:
%   J = Σ(k=0 to N-1) [‖y[k] - r[k]‖²_Q + ‖u[k]‖²_R + ‖u[k] - u[k-1]‖²_S]
%
% where:
%   y[k] = C·x[k]                    (measured output)
%   r[k]                              (reference trajectory)
%   u[k] ∈ ℝ^nu                       (control input)
%   N                                 (prediction horizon)
%   Φ = [A; A²; A³; ...; A^N]        (state transition matrix, stacked)
%   Ψ = toeplitz(B,A,N)               (input-to-state influence matrix)
%   C                                 (output selection matrix)
%   G = [g; A·g; A²·g; ...]          (disturbance propagation)
%
% The Hessian H captures:
%   - Tracking error cost: y'Qy (penalizes deviation from reference)
%   - Input effort cost: u'Ru (penalizes large control magnitudes)
%   - Smoothness cost: Δu'SΔu (penalizes jerky changes, reduces chattering)
%
% INPUTS:
%   A      ∈ ℝ^(n×n)     Discrete state transition matrix
%   B      ∈ ℝ^(n×nu)    Discrete input matrix
%   C      ∈ ℝ^(nr×n)    Output selection matrix (subset of states)
%   G      ∈ ℝ^(n×nd)    Disturbance matrix (e.g., gravity)
%   Qy     ∈ ℝ^(nr×nr)   Output tracking weight (positive semi-definite)
%          or ∈ ℝ scalar  (expanded to diagonal for SISO systems)
%   Qu     ∈ ℝ^(nu×nu)   Input effort weight (positive semi-definite)
%          or ∈ ℝ scalar  (expanded to diagonal for SISO systems)
%   N      ∈ ℤ⁺          Prediction horizon (number of steps)
%   c      ∈ {1,2}       Flag: 1 = with gravity term (altitude),
%                        2 = without gravity (attitude, planar)
%
% OUTPUTS:
%   H      ∈ ℝ^(N·nu × N·nu)   Hessian of QP (quadratic cost coefficient)
%   F1     ∈ ℝ^(N·nu × n)      Linear cost: coupling with initial state
%   F2     ∈ ℝ^(N·nu × nr)     Linear cost: coupling with reference traj
%   F3     ∈ ℝ^(N·nu)          Linear cost: constant input term
%   F4     ∈ ℝ^(N·nu)          Linear cost: gravity coupling (c=1 only)
%   phi    ∈ ℝ^(N·n × n)       State transition matrix (Φ, stacked)
%   psi    ∈ ℝ^(N·n × N·nu)    Input-to-state matrix (Ψ, stacked)
%   GMat   ∈ ℝ^(N·n × nd)      Disturbance propagation matrix (c=1 only)
%
% WHY THIS FORM:
%   - QP solver (quadprog) expects H and F in this standard form
%   - Pre-computing H, F avoids repeated matrix operations at each MPC cycle
%   - Offline computation amortizes cost across many samples
%   - Finite-horizon formulation tractable (~10-15 horizon steps per layer)
%
% REFERENCES:
%   - Boyd & Parikh, "Convex Optimization", Ch. 4 (QP formulation)
%   - Rawlings & Mayne, "Model Predictive Control" (MPC cost function)
%   - Franklin & Powell, "Digital Control of Dynamic Systems" (discretization)

Phi_i = A;
Aux_Psi_i = B;
auxG_i = G;

[n,nu] = size(B);
nr = size(C,1);
psi = [];
phi = [];
GMat = [];

H = 0;
F1 = 0;
F2 = 0;
F3 = 0;
F4 = 0;

if c == 1  % WITH GRAVITY DISTURBANCE (altitude control)
    for i = 1:N
        % Psi_i: input influence matrix for timestep i
        % Shape: [n × N·nu] — how i-th control affects state trajectory
        % Captures: u[0] → x[i], u[1] → x[i], ..., u[i] → x[i]
        Psi_i = [Aux_Psi_i, zeros(n,(N-i)*nu)];

        % G_i: disturbance (gravity) influence at timestep i
        % Shape: [n × (N-i)·nd] — cumulative gravity effect
        G_i = [auxG_i,zeros(n,(N-i)*nu)];

        % Sum gravity effects over all future steps (altitude dynamics)
        % Used in F4 term: cost penalty for gravity disturbance
        SumG_1 = sum(G_i(1,:));
        SumG_2 = sum(G_i(2,:));
        SumG = [SumG_1;SumG_2];

        % Stack matrices across horizon
        psi = [psi;Psi_i];
        phi = [phi;Phi_i];

        % Pi_i(i,nx,N): selector matrix extracting x[i] from stacked state
        % Decomposes multi-step tracking cost into per-timestep terms
        Pi_ny = Pi_i(i,nr,N);
        Pi_nu = Pi_i(i,nu,N);

        % H: Hessian, quadratic coefficient of cost
        % Term 1: ‖y[i] - r[i]‖²_Q = ‖C·x[i]‖²_Q
        %         → Ψ_i'C'QC Ψ_i (control-to-error sensitivity)
        % Term 2: ‖u[i]‖²_R (input effort penalty)
        % Factor 2: MATLAB quadprog convention (½u'Hu → 2u'Hu for direct form)
        H = H + 2*(Psi_i'*C'*Qy*C*Psi_i + (Pi_nu)'*Qu*Pi_nu);

        % F1: linear coupling with initial state x[0]
        % If x[0] ≠ 0, tracking cost increases by Ψ_i'C'QC Φ_i · x[0]
        % Represents how initial condition affects trajectory cost
        F1 = F1 + 2*(Psi_i'*C'*Qy*C*Phi_i);

        % F2: linear coupling with reference trajectory r[i]
        % Tracking cost: min ‖y[i] - r[i]‖²_Q = -2r[i]'Qy·y[i] + const
        % Extracts only reference-dependent terms
        F2 = F2 - 2*(Psi_i'*C'*Qy*Pi_ny);

        % F3: constant input effort term
        % From cost ‖u[i]‖²_R: contributes -2R·u[i] to linear term
        F3 = F3 - 2*((Pi_nu)'*Qu);

        % F4: gravity disturbance coupling
        % Altitude MPC only: gravity g acts as constant downward disturbance
        % Cost includes penalty for gravity effect on trajectory
        % This term forces optimizer to account for gravity in control
        F4 = F4 - 2*(Psi_i'*C'*Qy*C*SumG);

        % Advance to next timestep in recursion
        % Φ_{i+1} = Φ_i · A (matrix power accumulation)
        % Ψ_{i+1} = [A·Ψ_i | B] (shift and append B for new control input)
        % G_{i+1} = [A·G_i | G] (disturbance accumulation)
        Phi_i = Phi_i*A;
        Aux_Psi_i = [A*Aux_Psi_i B];
        auxG_i = [A*auxG_i G];
    end
else  % WITHOUT DISTURBANCE TERM (attitude & planar control)
    % Attitude MPC: no gravity term (angles and rates as states)
    % Planar MPC: gravity modeled as input constraint, not cost term
    for i = 1:N
        Psi_i = [Aux_Psi_i, zeros(n,(N-i)*nu)];
        psi = [psi;Psi_i];
        phi = [phi;Phi_i];

        Pi_ny = Pi_i(i,nr,N);
        Pi_nu = Pi_i(i,nu,N);

        % Identical to c=1 case, except F4 remains zero (no gravity penalty)
        % Cost function: J = Σ[‖y[i] - r[i]‖²_Q + ‖u[i]‖²_R]
        % (no feedthrough disturbance term)
        H = H + 2*(Psi_i'*C'*Qy*C*Psi_i + (Pi_nu)'*Qu*Pi_nu);
        F1 = F1 + 2*(Psi_i'*C'*Qy*C*Phi_i);
        F2 = F2 - 2*(Psi_i'*C'*Qy*Pi_ny);
        F3 = F3 - 2*((Pi_nu)'*Qu);
        F4 = 0;  % No gravity disturbance for attitude/planar layers

        Phi_i = Phi_i*A;
        Aux_Psi_i = [A*Aux_Psi_i B];
    end
end
