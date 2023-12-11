classdef GibbsAppellBodyEquations
properties (GetAccess = public, SetAccess = private)
    Jacobian sym;
    JacobianRate sym;
    PartialVelocity sym;
    PartialVelocityRate sym;
    MassMatrix sym;
    ForcingVector sym;
    Linearized LinearizedDynamicEquations;
end
methods (Access = public)
function obj = GibbsAppellBodyEquations(q,qd,u,ud,Ju,qds,Jdu,body) 
    obj.Jacobian = body.Twist.jacobian(qd);
    obj.JacobianRate = obj.jacobianRate(q,qds);
    obj.PartialVelocity = obj.Jacobian*Ju;
    obj.PartialVelocityRate = obj.JacobianRate*Ju + obj.Jacobian*Jdu;
    obj.MassMatrix = obj.massMatrix(body);
    obj.ForcingVector = obj.forcingVector(body,u);
    M = obj.MassMatrix;
    f = obj.ForcingVector;
    obj.Linearized = LinearizedDynamicEquations(M,f); 
end
end
methods (Access = private)
function Jd = jacobianRate(obj,q,qds)
    f = @(j)simplify(expand(jacobian(j,q)))*qds;
    J = obj.Jacobian;
    Jd = reshape(arrayfun(f,reshape(J,[],1)),size(J));
end
function M = massMatrix(obj,body)
    Vbar = obj.PartialVelocity;
    G = blkdiag(body.Inertia,body.Mass*eye(3));
    M = Vbar.'*G*Vbar;
end
function f = forcingVector(obj,body,u)
    Vbar = obj.PartialVelocity;
    Vdbar = obj.PartialVelocityRate;
    V = Vbar*u;
    wm = vec2skew(V(1:3));
    vm = vec2skew(V(4:6));
    ad = [
        wm,zeros(3);
        vm,wm
    ];
    G = blkdiag(body.Inertia,body.Mass*eye(3));
    F = body.ActiveForces;
    f = Vbar.'*F - Vbar.'*(G*Vdbar - ad.'*G*Vbar)*u;
end
end
end