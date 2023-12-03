function out = gibbsAppell(eomd)
arguments
    eomd (1,1) EquationsOfMotion;
end

qd = eomd.eomk.qd;
u = eomd.eomk.u;
W = eomd.eomk.Ju(:,1:eomd.eomk.k);
qds = W*u;
f = @(x)simplify(expand(subs(x,qd,qds)));
Wd = f(eomd.eomk.Jdu(:,1:eomd.eomk.k));
G = eomd.SpatialInertia;
J = eomd.Jacobian;
Jd = f(eomd.JacobianRate);
ad = f(eomd.TwistAdjoint);
Vbar = simplify(expand(J*W));
Vdbar = simplify(expand(Jd*W + J*Wd));

eomd.MassMatrix = simplify(expand(W.'*eomd.MassMatrix*W));
Qi = -Vbar.'*(G*Vdbar - ad.'*G*Vbar)*u;
eomd.InertialForces = simplify(expand(Qi));
eomd.ActiveForces = simplify(expand(W.'*eomd.ActiveForces));
eomd.FrictionForces = f(W.'*eomd.FrictionForces);
Q = eomd.InertialForces + eomd.ActiveForces + eomd.FrictionForces;
eomd.ForcingVector = simplify(expand(Q));
out = eomd;