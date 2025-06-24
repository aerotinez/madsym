function syso = observerLMIHinfUIO(sys, E, a)
% OBSERVERHINFUIO - H∞-robust Unknown Input Observer via LMIs
%
% Inputs:
%   sys   - ss object: dx = A x + B u, y = C x
%   E     - disturbance input matrix
%   alpha - optional convergence rate (default: 0)
%
% Output:
%   syso  - state-space observer: dot(x̂) = (A - LC)x̂ + Bu + Ly

    if nargin < 3
        a = 0; % decay rate
    end

    % Extract system
    A = sys.A;
    B = sys.B;
    C = sys.C;

    nx = size(A,1);
    ny = size(C,1);
    nd = size(E,2);

    % YALMIP variables
    P = sdpvar(nx, nx);
    R = sdpvar(nx, ny, 'full');
    gamma = sdpvar(1);

    ep = 1e-12;

    % LMI matrix for H∞ UIO
    Acl = A * P - R * C;
    M11 = Acl + Acl' + 2 * a * P;
    M12 = P * E;
    M13 = R;

    M22 = -gamma * eye(nd);
    M33 = -gamma * eye(ny);

    LMI = [
        M11,   M12,   M13;
        M12',  M22,   zeros(nd, ny);
        M13',  zeros(ny, nd), M33
    ];

    constraints = [P >= ep * eye(nx), LMI <= -ep * eye(nx + nd + ny), gamma >= ep];

    % Solve
    opts = sdpsettings('solver','sdpt3','verbose',false);
    optimize(constraints, gamma, opts);

    % Compute gain and observer system
    L = value(P) \ value(R);
    A_obs = A - L * C;
    B_obs = [B, L];  % Inputs: [u; y]
    C_obs = eye(nx);
    D_obs = zeros(nx, size(B_obs,2));

    syso = ss(A_obs, B_obs, C_obs, D_obs);
end
