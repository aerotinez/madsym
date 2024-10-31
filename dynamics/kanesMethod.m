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
    qk = q.partition();
    uk = u.partition();
    eomk = KinematicEquations(qk,kdes,uk).simplify();
    eomc = ConstraintEquations.empty(0,1);
    Jc = sym.empty(0,1);
    if ~isempty(cons)
        eomc = partition(cons).reformulate(eomk); 
        Jc = partitionJacobian(eomc);
    end
    eomd_list = arrayfun(@(b)bodyDynamics(b,eomk,F,Jc),bodies);
    eomv = MotionEquations.empty(0,1);
    if ~isempty(ades)
        eomv = auxiliaryEquations(v,ades,eomk,F);
    end
    eom = MechanicsEquations(eomk,eomd_list,eomc,eomv);
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

    eomd = -DynamicEquations(u,M,f0,f1,f2,inputs);

    if ~isempty(Jc)
        eomd = constrain(eomd,Jc); 
    end
end