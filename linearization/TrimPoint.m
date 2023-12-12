classdef TrimPoint
properties (GetAccess = public, SetAccess = private)
    x;
    x0; 
end
methods
function obj = TrimPoint(equations,inputs,q0,u0,v0,f0)
    arguments
        equations EquationsOfMotion;
        inputs (:,1) sym = sym.empty([0,1]);
        q0 (:,1) sym = zeros(size(equations.q),'sym');
        u0 (:,1) sym = zeros(size(equations.u),'sym');
        v0 (:,1) sym = zeros(size(equations.v),'sym');
        f0 (:,1) sym = sym.empty([0,1]);
    end

    obj.x = [
        equations.q;
        equations.u;
        equations.v;
        inputs;
        ];

    obj.x0 = [
        q0;
        u0;
        v0;
        f0;
        ];
end
end
end