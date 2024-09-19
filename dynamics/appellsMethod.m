function eom = appellsMethod(x,kdes,bodies,inputs,cons,ades)
    arguments
        x (1,1) StateVector;
        kdes (:,1) sym;
        bodies (:,1) Body;
        inputs (1,1) GeneralizedCoordinates = GeneralizedCoordinates();
        cons (:,1) ConstraintEquations = ConstraintEquations.empty(0,1);
        ades (:,1) sym = sym.empty(0,1);
    end
    q = x.Coordinates;
    uga = generalizedSpeeds(x.Speeds);
    v = x.Auxiliary; 
    xga = StateVector(q,uga,v);
    eomk = kinematics(q,kdes,uga,cons);
    eomd_list = arrayfun(@(b)bodyDynamics(b,eomk,inputs),bodies);
    eomc = ConstraintEquations.empty(0,1);
    if ~isempty(cons) && ~isempty(cons.Configuration)
        eomc = ConstraintEquations(x.Coordinates,cons.Configuration);
    end
    eoma = MotionEquations.empty(0,1);
    if ~isempty(ades)
        eoma = auxiliaryEquations(x,ades,eomk,inputs);
    end
    eom = MechanicsEquations(xga,eomk,eomd_list,eomc,eoma);
end

function uga = generalizedSpeeds(u)
    u0 = u.Trim(1:numel(u.Independent));
    ud0 = u.TrimRate(1:numel(u.Independent));
    uga = GeneralizedCoordinates(u.Independent,[],u0,ud0);
end

function eomk = kinematics(q,kdes,u,cons) 
    t = sym('t');
    qd = diff(q.All,t);

    eq = kdes;
    if ~isempty(cons)
        eq = [
            kdes;
            cons.Jacobian*qd
            ];
    end

    eomk = KinematicEquations(q,eq,u);
end

function eomd = bodyDynamics(body,eomk,inputs)
    t = sym('t');
    qd = diff(eomk.States.All,t);
    u = eomk.Inputs.All;

    V = body.Twist.reformulate(eomk);
    ad = V.adjoint();

    Vbar = V.partial(eomk);
    Vdbar = V.partialRate(eomk);

    G = blkdiag(body.Inertia,body.Mass.*eye(3));
    M = Vbar.'*G*Vbar;

    f0 = -Vbar.'*G*Vdbar*u;
    f1 = Vbar.'*ad.'*G*Vbar*u;

    T = Pose(body.ReferenceFrame,body.MassCenter);
    W = body.ActiveForces.vector(T);
    f2 = Vbar.'*subs(W,qd,eomk.ForcingVector);

    eomd = DynamicEquations(eomk.Inputs,M,f0,f1,f2,inputs);
end