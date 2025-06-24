function syso = observerPIHinf(sys, E, alpha)
    % H-infinity PI observer with convergence rate alpha
    arguments
        sys (:,:) ss
        E (:,:) double
        alpha (1,1) double {mustBeNonnegative} = 0;
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
    X  = sdpvar(nx, nx, 'symmetric');
    Kp = sdpvar(nx, ny, 'full');
    Ki = sdpvar(nx, ny, 'full');
    gamma = sdpvar(1,1);

    % Define terms
    Acl = A*X - Kp*C;
    Bw  = E;

    M11 = Acl + Acl' + 2*alpha*X;
    M12 = -Ki + (C*X)';
    M13 = X*Bw;

    % LMI block for H∞ + convergence rate
    LMI = [M11,      M12,         M13;
           M12',     zeros(ny),   zeros(ny,nw);
           M13',     zeros(nw,ny), -gamma*eye(nw)];

    % Constraints
    ep = 1e-6;
    Constraints = [X >= ep*eye(nx), gamma >= 1e-3, LMI <= -ep*eye(size(LMI))];
    options = sdpsettings('solver','sdpt3','verbose',true);

    % Minimize gamma
    optimize(Constraints, gamma, options);

    Xv = value(X);
    Lp = Xv \ value(Kp);
    Li = Xv \ value(Ki);

    % Observer system
    Ao = [A - Lp*C,   -Li;
          C,           zeros(ny)];
    Bo = [B;
          zeros(ny, nu)];
    Eo = [Lp;
         -eye(ny)];
    Co = [C, zeros(ny)];
    Do = zeros(ny, nu + ny);

    syso = ss(Ao, [Bo Eo], Co, Do);
end
