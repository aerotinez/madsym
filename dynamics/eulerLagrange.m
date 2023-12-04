function out = eulerLagrange(eomd)
arguments
    eomd (1,1) EquationsOfMotion;
end

q = eomd.eomk.q;
qd = eomd.eomk.qd;
qdd = eomd.eomk.qdd;
L = eomd.Lagrangian;
A = eomd.eomk.A;
Qa = eomd.ActiveForces;
Qf = eomd.FrictionForces;

dLdqd = jacobian(L,qd).';
dLdq = jacobian(L,q).';
Qi = diff(dLdqd,sym('t')) - dLdq;

if eomd.eomk.m > 0
    out = LagrangeEquations(eomd.eomk,eomd.bodies);
    lm = out.Multipliers;
    Qc = A.'*lm;
else
    out = eomd;
    Qc = 0.*q;
end

eoms = [
    Qi - Qc - Qa - Qf;
    diff(diff(eomd.eomk.hc));
    diff(eomd.eomk.nhc)
    ];

M = simplify(expand(jacobian(eoms,[qdd;lm])));
f = simplify(expand(-subs(eoms,[qdd;lm],0.*[qdd;lm])));

out.MassMatrix = M;
out.InertialForces = f - [Qc + Qa + Qf;zeros(length(lm),1)];
out.InertialForces = simplify(expand(out.InertialForces));
out.ActiveForces = Qa;
out.FrictionForces = Qf;
out.ForcingVector = f;