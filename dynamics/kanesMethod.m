function eom = kanesMethod(x,kdes,bodies,inputs,constraints)
    arguments
        x (1,1) StateVector;
        kdes (:,1) sym;
        bodies (:,1) Body;
        inputs (1,1) GeneralizedCoordinates = GeneralizedCoordinates();
        constraints (:,1) ConstraintEquations = ConstraintEquations.empty(0,1);
    end
    q = generalizedCoordinates(x.Coordinates);
    u = generalizedSpeeds(x.Speeds); 
    xk = StateVector(q,u);
    eomk = KinematicEquations(q,kdes,u);
    eomc = ConstraintEquations.empty(0,1);
    Jc = sym.empty(0,1);
    if ~isempty(constraints)
        eomc = validateConstraints(q,constraints,eomk);
        Jc = constraintJacobian(eomc,u);
    end

    eomd_list = arrayfun(@(b)bodyDynamics(b,eomk,inputs,Jc),bodies);
    eom = MechanicsEquations(xk,eomk,eomd_list,eomc);
end

function qk = generalizedCoordinates(q)
    q_all = [
        q.Independent;
        q.Dependent
        ];

    q0 = q.P*q.Trim;
    qd0 = q.P*q.TrimRate;

    qk = GeneralizedCoordinates(q_all,q.Dependent,q0,qd0);
end

function uk = generalizedSpeeds(u)
    u_all = [
        u.Independent;
        u.Dependent
        ];

    u0 = u.P*u.Trim;
    ud0 = u.P*u.TrimRate;

    uk = GeneralizedCoordinates(u_all,u.Dependent,u0,ud0);
end

function eomc = validateConstraints(q,constraints,eomk)
    t = sym('t');
    hc = constraints.Configuration;
    nhc = constraints.Jacobian*diff(eomk.States.All,t);
    nhc = nhc(numel(hc) + 1:end);
    eomc = ConstraintEquations(q,hc,nhc).reformulate(eomk);
end

function Jc = constraintJacobian(constraints,u)
    B = constraints.Jacobian;
    Bind = B(:,1:numel(u.Independent));
    Bdep = B(:,numel(u.Independent) + 1:end);
    Jc = -syminv(Bdep)*Bind;
end

function eomd = bodyDynamics(body,eomk,inputs,Jc)
    arguments
        body (1,1) Body;
        eomk (1,1) KinematicEquations;
        inputs (1,1) GeneralizedCoordinates = GeneralizedCoordinates();
        Jc sym = sym.empty(0,1);
    end
    t = sym('t');
    q = eomk.States;
    qd = diff(q.All,t);
    u = eomk.Inputs;

    twist = body.Twist.reformulate(eomk);

    w = twist.angVel();

    V = [
        w;
        twist.linVel();
        ];
        
    Vbar = simplify(expand(jacobian(V,u.All)));
    fVdbar = @(vbar)jacobian(vbar,q.All)*eomk.ForcingVector;
    Vdbar = reshape(arrayfun(fVdbar,reshape(Vbar,[],1)),size(Vbar));
    adw = blkdiag(vec2skew(w),zeros(3));

    G = blkdiag(body.Inertia,body.Mass.*eye(3));
    M = -Vbar.'*G*Vbar;
    
    f0 = Vbar.'*G*Vdbar*u.All;
    f1 = Vbar.'*adw*G*Vbar*u.All;

    I = eye(3);
    N = zeros(3);
    T = Pose(body.ReferenceFrame,body.MassCenter);
    mb = [I,N]*body.ActiveForces.vector(T);
    fi = [N,I]*body.ActiveForces.vector();
    f2 = -Vbar.'*simplify(expand(subs([mb;fi],qd,eomk.ForcingVector)));

    eomd = DynamicEquations(u,M,f0,f1,f2,inputs);

    if ~isempty(Jc)
        eomd_ind = independentBodyDynamics(eomd,Jc);
        eomd_dep = dependentBodyDynamics(eomd,Jc);
        eomd = eomd_ind + eomd_dep;
    end
end

function eomd_ind = independentBodyDynamics(eomd,Jc)
    n = numel(eomd.States.All);
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