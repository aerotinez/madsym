classdef KinematicEquations
properties (GetAccess = public, SetAccess = private)
    n; % number of generalized coordinates
    l; % number of holonomic constraints
    m; % number of nonholonomic constraints
    k; % number of independent equations
    q; % generalized coordinates
    qd; % generalized rates
    qdd; % generalized accelerations
    u; % quasi-velocities
    ud; % quasi-accelerations
    Jqd; % kinematic Jacobian
    Ju; % quasi-velocity Jacobian
    Jdqd; % kinematic Jacobian derivative
    Jdu; % quasi-acceleration Jacobian
    hc; % holonomic constraints
    nhc; % nonholonomic constraints
    A; % constraint matrix
end
methods (Access = public)
function obj = KinematicEquations(q,u,kdes,options)
    arguments
        q (:,1) sym;
        u (:,1) sym;
        kdes (:,1) sym;
        options.hc (:,1) sym = sym.empty();
        options.nhc (:,1) sym = sym.empty();
    end
    obj.q = q;
    obj.u = u;
    obj.n = numel(q);
    obj.l = numel(options.hc);
    obj.m = numel([options.hc;options.nhc]);
    obj.k = obj.n - obj.m;
    obj.qd = diff(obj.q);
    obj.qdd = diff(obj.qd);
    obj.ud = diff(obj.u);
    obj.hc = options.hc;
    obj.nhc = [
        simplify(expand(diff(obj.hc,sym('t'))));
        options.nhc
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
    obj.Jdqd = simplify(expand(diff(obj.Jqd,sym('t'))));
    obj.Jdu = simplify(expand(-obj.Ju*obj.Jdqd*obj.Ju));
end
end
end