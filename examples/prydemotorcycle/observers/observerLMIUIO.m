function syso = observerLMIUIO(sys, E)
% OBSERVERLMIUIO - Full-state Unknown Input Observer (UIO) via LMI
%
% Inputs:
%   sys - state-space system (ss object), representing:
%         dx = A x + B u, y = C x
%   E   - disturbance input matrix (same number of rows as A)
%
% Output:
%   syso - observer as ss object: dot(x̂) = (A - LC)x̂ + B u + L y

    % Extract system matrices
    A = sys.A;
    B = sys.B;
    C = sys.C;

    nx = size(A,1);
    nu = size(B,2);
    nd = size(E,2);
    ny = size(C,1);

    % YALMIP variables
    P = sdpvar(nx, nx);
    R = sdpvar(nx, ny, 'full');
    ep = 1e-5;

    % Stability condition: (A - LC)' P + P (A - LC) < 0
    Acl = A * P - R * C;
    LMI_stability = Acl + Acl' <= -ep * eye(nx);

    % Decoupling condition: (A - LC) * E = 0
    LMI_decoupling = Acl * E == 0;

    % Constraints
    constraints = [P >= ep * eye(nx), LMI_stability, LMI_decoupling];

    % Solve LMI
    opts = sdpsettings('solver','sdpt3','verbose',true);
    optimize(constraints, [], opts);

    % Compute gain
    Pval = value(P);
    Rval = value(R);
    L = Pval \ Rval;

    % Build observer system: dot(x̂) = (A - LC)x̂ + Bu + Ly
    A_obs = A - L * C;
    B_obs = [B, L];                     % Inputs: [u; y]
    C_obs = eye(nx);                    % Outputs: estimated full state
    D_obs = zeros(nx, nu + ny);

    syso = ss(A_obs, B_obs, C_obs, D_obs);
end