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
p = numel(eomd.AuxilarySpeeds);

eomaux = f(eomd.AuxilaryEquations);
M = simplify(expand(W.'*eomd.MassMatrix*W));
Maux = simplify(expand(jacobian(eomaux,diff(eomd.AuxilarySpeeds))));

eomd.MassMatrix = [
    M,zeros(eomd.eomk.k,p);
    zeros(p,eomd.eomk.k),Maux
    ];

Qi = [
    -Vbar.'*(G*Vdbar - ad.'*G*Vbar)*u;
    -subs(eomaux,diff(eomd.AuxilarySpeeds),zeros(p,1))
    ];

eomd.InertialForces = simplify(expand(Qi));

eomd.ActiveForces = [
    simplify(expand(W.'*eomd.ActiveForces))
    zeros(p,1)
    ];

eomd.FrictionForces = [
    f(W.'*eomd.FrictionForces);
    zeros(p,1)
    ];

Q = eomd.InertialForces + eomd.ActiveForces + eomd.FrictionForces;
eomd.ForcingVector = simplify(expand(Q));
out = eomd;