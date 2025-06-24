function syso = observerPI(sys, a)
    arguments
        sys (:,:) ss;
        a (1,1) double = 0;
    end

    yalmip('clear');

    % System matrices
    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;

    nx = size(A,1);  % state dimension
    nu = size(B,2);  % input dimension
    ny = size(C,1);  % output dimension

    % Decision variables
    X = sdpvar(nx, nx, 'symmetric');
    Kp = sdpvar(nx, ny, 'full');
    Ki = sdpvar(nx, ny, 'full');

    % LMI for stability of augmented system
    M11 = A*X - Kp*C;
    M12 = -Ki + (C*X)';
    LMI = [M11 + M11' + 2*a*X, M12;
           M12',       zeros(ny)];

    % Constraints
    ep = 1e-6;
    Constraints = [X >= ep*eye(nx), LMI <= -ep*eye(nx + ny)];

    % Solve
    opts = sdpsettings('solver', 'sdpt3', 'verbose', true);
    optimize(Constraints, [], opts);

    % Recover gains
    Xv = value(X);
    Lp = Xv \ value(Kp);
    Li = Xv \ value(Ki);

    % Build state-space observer model
    Ao = [A - Lp*C, -Li;
          C,        zeros(ny)];

    % Input matrices for [u; y]
    Bu = [B;
          zeros(ny, nu)];

    Ey = [Lp;
         -eye(ny)];

    Eu = [zeros(nx, nu);
         -D];

    % Combine all inputs: [u; y; u] → really means [u; y], but y includes D*u
    B_all = [Bu Ey Eu];
    Co = [C, zeros(ny)];
    Do = zeros(ny, size(B_all, 2));

    syso = ss(Ao, B_all, Co, Do);
end
