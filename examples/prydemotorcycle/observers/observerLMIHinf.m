function syso = observerLMIHinf(sys, E, a)
    arguments
        sys (:,:) ss;
        E (:,:) double;
        a (1,1) double = 1;
    end

    clear('yalmip');

    % Extract system matrices
    A = sys.A;
    B = sys.B;
    C = sys.C;

    nx = size(A,1);
    ny = size(C,1);
    nd = size(E,2);
    nu = size(B,2);

    % YALMIP variables
    P = sdpvar(nx, nx);
    R = sdpvar(nx, ny, 'full');
    g = sdpvar(1, 1);

    % Construct LMI
    M11 = (A*P - R*C) + (A*P - R*C)' + 2*a*P;
    M12 = P*E;
    M13 = R;
    M22 = -g * eye(nd);
    M33 = -g * eye(ny);

    LMI = [M11, M12, M13;
           M12', M22, zeros(nd, ny);
           M13', zeros(ny, nd), M33];

    ep = 1e-6;
    F = [P >= ep*eye(nx), LMI <= -ep*eye(nx + nd + ny), g >= ep];

    opts = sdpsettings('solver','sdpt3','verbose',true);
    optimize(F, g, opts);

    % Recover gain
    g_opt = value(g);
    L = value(P) \ value(R);

    % Construct observer system: dot(x̂) = (A - LC)x̂ + Bu + L y
    A_obs = A - L*C;
    B_obs = [B, L];  % Inputs: [u; y]
    C_obs = eye(nx); % Outputs: estimated full state
    D_obs = zeros(nx, nu + ny);

    syso = ss(A_obs, B_obs, C_obs, D_obs);
end