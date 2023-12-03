classdef KinematicEquations
properties (GetAccess = public, SetAccess = private)
    n; % number of generalized coordinates
    l; % number of holonomic constraints
    m; % number of nonholonomic constraints
    k; % number of independent equations
    q; % generalized coordinates
    qd; % generalized rates
    u; % quasi-velocities
    Jqd; % kinematic Jacobian
    Ju; % quasi-velocity Jacobian
    hc; % holonomic constraints
    nhc; % nonholonomic constraints
    A; % constraint matrix
end
methods (Access = public)
function obj = KinematicEquations(q,u,kdes,hc,nhc)
    arguments
        q (:,1) sym;
        u (:,1) sym;
        kdes (:,1) sym;
        hc (:,1) sym = sym.empty();
        nhc (:,1) sym = sym.empty();
    end
    obj.q = q;
    obj.u = u;
    obj.n = numel(q);
    obj.l = numel(hc);
    obj.m = numel([hc;nhc]);
    obj.k = obj.n - obj.m;
    obj.qd = diff(obj.q);
    obj.hc = hc;
    obj.nhc = [
        simplify(expand(diff(hc)));
        nhc
    ];
    obj.A = jacobian(obj.nhc,obj.qd);
    switch length(obj.u)
    case obj.n
        obj.Jqd = jacobian(obj.u - kdes,obj.qd);
    case obj.k
        obj.Jqd = [
            jacobian(obj.u - kdes,obj.qd);
            obj.A;
        ];
    otherwise
        error('Invalid number of quasi-velocities');
    end
    obj.Ju = simplify(expand(inv(obj.Jqd)));
end
end
end