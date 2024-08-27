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
    eomd_list = bodyDynamics(bodies,eomk,inputs);
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

function eomd_list = bodyDynamics(bodies,eomk,inputs)
    eomd_list = arrayfun(@(b)b.dynamics(eomk,inputs),bodies);
end