function syso = observerLMI(sys, a)
    arguments
        sys (:,:) ss;
        a (1,1) double {mustBePositive} = 1;
    end

    clear('yalmip');

    % Extract system matrices
    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;

    nx = size(A,1);
    nu = size(B,2);
    ny = size(C,1);

    % LMI variables
    P = sdpvar(nx, nx, 'symmetric');
    R = sdpvar(nx, ny, 'full');
    ep = 1e-1;

    % Stability LMI: (A - LC)'P + P(A - LC) + 2aP < 0
    F = [
        P >= 0;
        (A*P - R*C) + (A*P - R*C)' + 2*a*P <= 0
    ];

    opts = sdpsettings('solver','sdpt3','verbose',true);
    optimize(F, [], opts);

    % Recover observer gain
    Pval = value(P);
    Rval = value(R);
    L = Pval \ Rval;
    disp(L);

    % Observer state-space model
    %   dot(x̂) = (A - LC)x̂ + Bu + L(y - D*u)
    % Inputs: [u; y]
    A_obs = A - L * C;
    B_u = B - L * D;         % adjust for D*u in the measurement
    B_y = L;                 % gain on y
    B_obs = [B_u, B_y];      % inputs: [u; y]

    C_obs = eye(nx);         % outputs: full state estimate
    D_obs = zeros(nx, nu + ny);

    syso = ss(A_obs, B_obs, C_obs, D_obs);
end
