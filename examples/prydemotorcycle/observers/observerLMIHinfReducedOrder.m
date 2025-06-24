function syso = observerLMIHinfReducedOrder(sys, E, a)
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

    % Determine measured and unmeasured state indices
    idxm = find(any(C,1));       % measured state indices
    idx = 1:nx;
    idxu = setdiff(idx, idxm);   % unmeasured state indices

    nu = length(idxu);           % number of unmeasured states

    % Partition matrices
    Auu = A(idxu, idxu);         % unmeasured-unmeasured
    Amu = A(idxu, idxm);         % measured-to-unmeasured (corrected)
    Bu  = B(idxu, :);            % input effect on unmeasured
    Eu  = E(idxu, :);            % disturbance effect
    Cm  = C(:, idxu);            % how unmeasured affect outputs

    % YALMIP variables
    P = sdpvar(nu, nu);
    R = sdpvar(nu, ny, 'full');
    g = sdpvar(1);

    % H-infinity LMI
    M11 = Auu*P - R*Cm + P*Auu' - Cm'*R' + 2*a*P;
    M12 = P*Eu;
    M13 = R;
    M22 = -g * eye(nd);
    M33 = -g * eye(ny);

    LMI = [M11, M12, M13;
           M12', M22, zeros(nd, ny);
           M13', zeros(ny, nd), M33];

    ep = 1e-6;
    F = [P >= ep*eye(nu), LMI <= -ep*eye(nu + nd + ny), g >= ep];

    opts = sdpsettings('solver','sdpt3','verbose',false);
    optimize(F, g, opts);

    % Observer gain
    L = value(P) \ value(R);

    % Observer system: dot(x̂_u) = A_obs x̂ + B_obs [y; u]
    Aobs = Auu - L * Cm;
    Bobs = [Amu + L, Bu];           % inputs: [y; u]
    Cobs = eye(nu);                 % output: unmeasured state estimates
    Dobs = zeros(nu, ny + size(B,2));

    % Return system
    syso = ss(Aobs, Bobs, Cobs, Dobs);
end