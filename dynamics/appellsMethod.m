function eom = appellsMethod(q,u,kdes,bodies,F,cons,v,ades)
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

    uga = u.independent();
    eomk = kinematics(q,kdes,uga,cons);

    eomd_list = arrayfun(@(b)bodyDynamics(eomk,b,F),bodies);

    eomc = ConstraintEquations.empty(0,1);
    if ~isempty(cons) && ~isempty(cons.Configuration)
        eomc = ConstraintEquations(q,cons.Configuration);
    end

    eomv = MotionEquations.empty(0,1);
    if ~isempty(ades)
        eomv = auxiliaryEquations(v,ades,eomk,F);
    end

    eom = MechanicsEquations(eomk,eomd_list,eomc,eomv);
end

function eomk = kinematics(q,kdes,u,cons) 
    eqns = kdes;

    if ~isempty(cons)
        eqns = [
            kdes;
            cons.Jacobian*q.rate()
            ];
    end

    eomk = KinematicEquations(q,eqns,u);
end

function eomd = bodyDynamics(eomk,body,inputs)
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
    W = body.ActiveForces.vector(T);
    f2 = Vbar.'*subs(W,eomk.States.rate,eomk.ForcingVector);

    eomd = DynamicEquations(eomk.Inputs,M,f0,f1,f2,inputs);
end