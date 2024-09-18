function eom = appellsMethod(x,kdes,bodies,inputs,constraints)
    arguments
        x (1,1) StateVector;
        kdes (:,1) sym;
        bodies (:,1) Body;
        inputs (1,1) GeneralizedCoordinates = GeneralizedCoordinates();
        constraints (:,1) ConstraintEquations = ConstraintEquations.empty(0,1);
    end
    q = x.Coordinates;
    uga = generalizedSpeeds(x.Speeds); 
    xga = StateVector(q,uga);
    eomk = kinematics(q,kdes,uga,constraints);
    eomd_list = arrayfun(@(b)bodyDynamics(b,eomk,inputs),bodies);
    eomc = ConstraintEquations.empty(0,1);
    if ~isempty(constraints) && ~isempty(constraints.Configuration)
        eomc = ConstraintEquations(x.Coordinates,constraints.Configuration);
    end
    eom = MechanicsEquations(xga,eomk,eomd_list,eomc);
end

function uga = generalizedSpeeds(u)
    u0 = u.Trim(1:numel(u.Independent));
    ud0 = u.TrimRate(1:numel(u.Independent));
    uga = GeneralizedCoordinates(u.Independent,[],u0,ud0);
end

function eomk = kinematics(q,kdes,u,constraints) 
    t = sym('t');
    qd = diff(q.All,t);

    eq = kdes;
    if ~isempty(constraints)
        eq = [
            kdes;
            constraints.Jacobian*qd
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