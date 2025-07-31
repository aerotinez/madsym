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
        eomc = simplify(partition(cons).reformulate(eomk)); 
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

    V = body.Twist.reformulate(eomk);
    ad = V.adjoint();

    Vbar = V.partial(eomk);
    Vdbar = V.partialRate(eomk);

    G = blkdiag(body.Inertia,body.Mass.*eye(3));
    M = Vbar.'*G*Vbar;

    u = eomk.Inputs.state;
    f0 = -Vbar.'*G*Vdbar*u;
    f1 = Vbar.'*ad.'*G*Vbar*u;

    T = Pose(body.ReferenceFrame,body.MassCenter);
    W = simplify(expand(body.ActiveForces.vector(T)));
    f2 = Vbar.'*subs(W,eomk.States.rate,eomk.ForcingVector);

    eomd = -DynamicEquations(eomk.Inputs,M,f0,f1,f2,inputs);

    if ~isempty(Jc)
        eomd = constrain(eomd,Jc); 
    end
end