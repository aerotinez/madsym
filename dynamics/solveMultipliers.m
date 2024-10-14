function lm = solveMultipliers(eomk,eomd_list,cons,v)
    arguments
        eomk (1,1) KinematicEquations;
        eomd_list (:,1) DynamicEquations;
        cons (:,1) ConstraintEquations;
        v (:,1) DynamicVariable = DynamicVariable.empty(0,1);
    end
    eomd = sum(eomd_list);

    x = [
        eomk.States;
        eomd.States;
        eomd.Inputs;
        v;
        ];

    u = eomk.Inputs.state;

    Md = subsTrim(eomd.MassMatrix,x);
    fd = subsTrim(eomd.ForcingVector,x);
    f0 = subsTrim(eomd.f0,x);
    f1 = eomd.f1;
    f2 = subsTrim(eomd.f2,x);
    A = subsTrim(cons.Jacobian,x);
    Ad = subsTrim(cons.JacobianRate,x);
    
    if rank(Md) == size(Md,1)
        lm = solveMultipliersFast(u,Md,f0,f1,f2,A,Ad);
        return;
    end
    lm = solveMultipliersSlow(u,Md,fd,A,Ad);
end

function args = solveMultipliersFast(u,Md,f0,f1,f2,A,Ad)
    Minv = syminv(Md);
    Sinv = syminv(A*Minv*A.');
    M = zeros(size(Md),"sym");
    C = A.'*Sinv*A*Minv;
    f0 = C*f0;
    f2 = C*f2 + A.'*Sinv*Ad*u;
    args = {M,f0,f1,f2};
end

function args = solveMultipliersSlow(u,Md,fd,A,Ad)
    M = [
        Md,-A.';
        A,zeros(size(A,1))
        ];

    f = [
        fd;
        -Ad*u
        ];

    sol = syminv(M)*f;
    lm = sol(size(Md,1) + 1:end);
    M = zeros(size(Md),"sym");
    f0 = A.'*lm;
    f1 = zeros(size(f0),"sym");
    f2 = zeros(size(f0),"sym");
    args = {M,f0,f1,f2};
end