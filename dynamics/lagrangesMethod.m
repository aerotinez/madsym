function [eom,lm] = lagrangesMethod(q,u,kdes,bodies,F,cons,v,ades)
    arguments
        q (:,1) DynamicVariable;
        u (:,1) DynamicVariable;
        kdes (:,1) sym;
        bodies (:,1) Body;
        F (:,1) DynamicVariable = DynamicVariable.empty(0,1);
        cons (:,1) ConstraintEquations = ConstraintEquations.empty(0,1);
        v (:,1) DynamicVariable = DynamicVariable.empty(0,1);
        ades (:,1) sym = sym.empty(0,1);
    end

    ql = [
        q.independent();
        q.dependent()
        ];

    ul = [
        u.independent();
        u.dependent()
        ];

    eomk = KinematicEquations(ql,kdes,ul).simplify();
    eomc = ConstraintEquations.empty(0,1);
    A = sym.empty(0,1);
    if ~isempty(cons)
        eomc = validateConstraints(q,cons,eomk);
        eomc = simplify(eomc);
        A = constraintJacobian(eomc,ul);
    end

    eomd_list = arrayfun(@(b)bodyDynamics(b,eomk,F,A),bodies);

    eomv = MotionEquations.empty(0,1);
    if ~isempty(ades)
        eomv = auxiliaryEquations(v,ades,eomk,F);
    end

    eom = MechanicsEquations(eomk,eomd_list,eomc,eomv);

    if nargout == 2
        lm = solveMultipliers(eomk,eomd_list,eomc,v);
    end
end

function eomc = validateConstraints(q,cons,eomk)
    hc = cons.Configuration;
    nhc = cons.Jacobian*q.rate;
    nhc = nhc(numel(hc) + 1:end);
    eomc = ConstraintEquations(eomk.States,hc,nhc).reformulate(eomk);
end

function A = constraintJacobian(cons,u)
    B = cons.Jacobian;
    Bind = B(:,1:numel(u.independent));
    Bdep = B(:,numel(u.independent) + 1:end);
    A = -syminv(Bdep)*Bind;
end

function eomd = bodyDynamics(body,eomk,inputs,A)
    arguments
        body (1,1) Body;
        eomk (1,1) KinematicEquations;
        inputs (:,1) DynamicVariable = DynamicVariable.empty(0,1);
        A sym = sym.empty(0,1);
    end
    V = body.Twist.vector;
    G = blkdiag(body.Inertia,body.Mass.*eye(3));
    L = (1/2).*V.'*G*V;

    dLdqd = jacobian(L,eomk.States.rate).';
    dLdq = jacobian(L,eomk.States.state).';
    t = sym('t');
    qdd = diff(eomk.States.rate,t);

    ovars = [
        qdd;
        eomk.States.rate;
        ];

    nvars = [
        eomk.Inputs.rate;
        eomk.Inputs.state;
        ];

    eqns = subs(diff(dLdqd,t) - dLdq,ovars,nvars);
    [M,f0] = massMatrixForm(eqns,eomk.Inputs.state);

    f1 = zeros(size(f0),"sym");

    J = body.Twist.jacobian(eomk.States);
    T = Pose(body.ReferenceFrame,body.MassCenter);
    W = body.ActiveForces.vector(T);
    f2 = J.'*subs(W,eomk.States.rate,eomk.ForcingVector);

    eomd = DynamicEquations(eomk.Inputs,M,f0,f1,f2,inputs);

    if ~isempty(A)
        eomd_ind = independentBodyDynamics(eomd,A);
        eomd_dep = dependentBodyDynamics(eomd,A);
        eomd = eomd_ind + eomd_dep;
    end
end

function eomd_ind = independentBodyDynamics(eomd,Jc)
    n = numel(eomd.States);
    m = size(Jc,1);
    k = n - m;
    M = eomd.MassMatrix(1:k,:);
    f0 = eomd.f0(1:k);
    f1 = eomd.f1(1:k);
    f2 = eomd.f2(1:k);
    eomd_ind = DynamicEquations(eomd.States,M,f0,f1,f2,eomd.Inputs);
end

function eomd_dep = dependentBodyDynamics(eomd,Jc)
    m = size(Jc,2);
    M = Jc.'*eomd.MassMatrix(m + 1:end,:);
    f0 = Jc.'*eomd.f0(m + 1:end);
    f1 = Jc.'*eomd.f1(m + 1:end);
    f2 = Jc.'*eomd.f2(m + 1:end);
    eomd_dep = DynamicEquations(eomd.States,M,f0,f1,f2,eomd.Inputs);
end

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