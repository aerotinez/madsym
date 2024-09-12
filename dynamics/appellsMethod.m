function eom = appellsMethod(x,kdes,bodies,inputs,constraints)
    arguments
        x (1,1) StateVector;
        kdes (:,1) sym;
        bodies (:,1) Body;
        inputs (:,1) sym = sym.empty(0,1);
        constraints (:,1) ConstraintEquations = ConstraintEquations.empty(0,1);
    end
    q = x.Coordinates.All;
    uga = GeneralizedCoordinates(x.Speeds.Independent);
    xga = StateVector(x.Coordinates,uga);
    eomk = kinematics(q,kdes,uga.Independent,constraints);
    eomd_list = arrayfun(@(b)bodyDynamics(b,eomk,inputs),bodies);
    eomc = ConstraintEquations.empty(0,1);
    if ~isempty(constraints) && ~isempty(constraints.Configuration)
        eomc = ConstraintEquations(x.Coordinates,constraints.Configuration);
    end
    eom = MechanicsEquations(xga,eomk,eomd_list,eomc);
end

function eomk = kinematics(q,kdes,u,constraints)
    arguments
        q (:,1) sym;
        kdes (:,1) sym;
        u (:,1) sym;
        constraints (:,1) ConstraintEquations = ConstraintEquations.empty(0,1);
    end
    if ~isempty(constraints) && ~isequal(constraints.Velocity.States,q)
        error("Constraint and StateVector coordinates do not match.");
    end
    t = sym('t');
    qd = diff(q,t);

    eq = kdes;
    if ~isempty(constraints)
        eq = [
            kdes;
            constraints.Velocity.MassMatrix*qd
            ];
    end

    eomk = KinematicEquations(q,eq,u);
end

function eomd = bodyDynamics(body,eomk,inputs)
    qd = eomk.Rates;
    u = eomk.Inputs;

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

    eomd = DynamicEquations(u,M,f0,f1,f2,inputs);
end