function out = gibbsAppell(eomd)
arguments
    eomd (1,1) EquationsOfMotion;
end

qd = eomd.eomk.qd;
u = eomd.eomk.u;
W = eomd.eomk.Ju(:,1:eomd.eomk.k);
qds = W*u;
f = @(x)subs(x,qd,qds);
Ju = eomd.eomk.Ju;
Jdqd = subs(eomd.eomk.Jdqd,qd,qds);
Jdu = -Ju*Jdqd*Ju;
Wd = Jdu(:,1:eomd.eomk.k);
G = eomd.SpatialInertia;
J = eomd.Jacobian;
Jd = f(eomd.JacobianRate);
ad = f(eomd.TwistAdjoint);
Vbar = J*W;
Vdbar = Jd*W + J*Wd;
p = numel(eomd.AuxiliarySpeeds);

eomaux = f(eomd.AuxiliaryEquations);
M = W.'*eomd.MassMatrix*W;
Maux = jacobian(eomaux,diff(eomd.AuxiliarySpeeds));

eomd.MassMatrix = [
    M,zeros(eomd.eomk.k,p);
    zeros(p,eomd.eomk.k),Maux
    ];

Qi = [
    -Vbar.'*(G*Vdbar - ad.'*G*Vbar)*u;
    -subs(eomaux,diff(eomd.AuxiliarySpeeds),zeros(p,1))
    ];

eomd.InertialForces = Qi;

eomd.ActiveForces = [
    W.'*eomd.ActiveForces
    zeros(p,1)
    ];

eomd.FrictionForces = [
    f(W.'*eomd.FrictionForces);
    zeros(p,1)
    ];

Q = eomd.InertialForces + eomd.ActiveForces + eomd.FrictionForces;
eomd.ForcingVector = Q;
out = eomd;