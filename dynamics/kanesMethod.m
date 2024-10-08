function eom = kanesMethod(q,u,kdes,bodies,F,cons,v,ades)
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

    qk = [
        q.independent();
        q.dependent()
        ];

    uk = [
        u.independent();
        u.dependent()
        ];

    eomk = KinematicEquations(qk,kdes,uk).simplify();
    eomc = ConstraintEquations.empty(0,1);
    Jc = sym.empty(0,1);
    if ~isempty(cons)
        eomc = validateConstraints(q,cons,eomk);
        eomc = simplify(eomc);
        Jc = constraintJacobian(eomc,uk);
    end

    eomd_list = arrayfun(@(b)bodyDynamics(b,eomk,F,Jc),bodies);

    eomv = MotionEquations.empty(0,1);
    if ~isempty(ades)
        eomv = auxiliaryEquations(v,ades,eomk,F);
    end

    eom = MechanicsEquations(eomk,eomd_list,eomc,eomv);
end

function eomc = validateConstraints(q,cons,eomk)
    hc = cons.Configuration;
    nhc = cons.Jacobian*q.rate;
    nhc = nhc(numel(hc) + 1:end);
    eomc = ConstraintEquations(eomk.States,hc,nhc).reformulate(eomk);
end

function Jc = constraintJacobian(cons,u)
    B = cons.Jacobian;
    Bind = B(:,1:numel(u.independent));
    Bdep = B(:,numel(u.independent) + 1:end);
    Jc = -syminv(Bdep)*Bind;
end

function eomd = bodyDynamics(body,eomk,inputs,Jc)
    arguments
        body (1,1) Body;
        eomk (1,1) KinematicEquations;
        inputs (:,1) DynamicVariable = DynamicVariable.empty(0,1);
        Jc sym = sym.empty(0,1);
    end
    q = eomk.States;
    u = eomk.Inputs;

    twist = body.Twist.reformulate(eomk);

    w = twist.angVel();

    V = [
        w;
        twist.linVel();
        ];
        
    Vbar = simplify(expand(jacobian(V,u.state)));
    fVdbar = @(vbar)jacobian(vbar,q.state)*eomk.ForcingVector;
    Vdbar = reshape(arrayfun(fVdbar,reshape(Vbar,[],1)),size(Vbar));
    adw = blkdiag(vec2skew(w),zeros(3));

    G = blkdiag(body.Inertia,body.Mass.*eye(3));
    M = -Vbar.'*G*Vbar;
    
    f0 = Vbar.'*G*Vdbar*u.state;
    f1 = Vbar.'*adw*G*Vbar*u.state;

    I = eye(3);
    N = zeros(3);
    T = Pose(body.ReferenceFrame,body.MassCenter);
    mb = [I,N]*body.ActiveForces.vector(T);
    fi = [N,I]*body.ActiveForces.vector();
    f2 = -Vbar.'*subs([mb;fi],q.rate,eomk.ForcingVector);

    eomd = DynamicEquations(u,M,f0,f1,f2,inputs);

    if ~isempty(Jc)
        eomd_ind = independentBodyDynamics(eomd,Jc);
        eomd_dep = dependentBodyDynamics(eomd,Jc);
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