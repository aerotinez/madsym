function syso = observerP2IHinf(sys, E, alpha)
    % Robust H∞ P2I observer with convergence rate constraint
    arguments
        sys (:,:) ss
        E (:,:) double
        alpha (1,1) double {mustBePositive}
    end

    yalmip('clear');

    % System matrices
    A = sys.A;
    B = sys.B;
    C = sys.C;

    nx = size(A,1);
    nu = size(B,2);
    ny = size(C,1);
    nw = size(E,2);

    % Variables
    X   = sdpvar(nx, nx, 'symmetric');
    Kp1 = sdpvar(nx, ny, 'full');
    Kp2 = sdpvar(nx, ny, 'full');
    gamma = sdpvar(1,1);

    % Closed-loop augmented error dynamics
    Acl = A*X - Kp1*C;
    M11 = Acl + Acl' + 2*alpha*X;
    M12 = -Kp2 + (C*X)';
    M13 = X*E;

    % LMI for H∞ + convergence rate
    LMI = [M11,     M12,         M13;
           M12',    zeros(ny),   zeros(ny,nw);
           M13',    zeros(nw,ny), -gamma*eye(nw)];

    ep = 1e-6;
    Constraints = [
        X >= ep*eye(nx), ...
        gamma >= 1e-3, ...
        LMI <= -ep*eye(size(LMI))
    ];

    options = sdpsettings('solver','sdpt3','verbose',0);
    optimize(Constraints, gamma, options);

    gamma_opt = value(gamma);
    Xv = value(X);
    Lp1 = Xv \ value(Kp1);
    Lp2 = Xv \ value(Kp2);

    % Build observer system
    Ao = [A - Lp1*C, -Lp2;
          C,         zeros(ny)];
    Bo = [B;
          zeros(ny, nu)];
    Eo = [Lp1;
         -eye(ny)];
    Co = [C, zeros(ny)];
    Do = zeros(ny, nu + ny);

    syso = ss(Ao, [Bo Eo], Co, Do);
end
