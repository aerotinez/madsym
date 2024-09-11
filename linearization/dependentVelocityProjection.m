function [C1,C2] = dependentVelocityProjection(states,constraints)
    arguments
        states (1,1) StateVector;
        constraints (1,1) ConstraintEquations;
    end
    fv = velocityConstraints(constraints);
    Jq = jacobian(fv,states.Coordinates.All);
    Ju = jacobian(fv,states.Speeds.All);
    Pui = states.Speeds.Pind;
    Pud = states.Speeds.Pdep;
    I = eye(numel(states.Speeds.All));
    C1 = -Pud*syminv(Ju*Pud)*Jq;
    C2 = (I - Pud*syminv(Ju*Pud)*Ju)*Pui;
end

function fv = velocityConstraints(cons)
    xd = cons.Velocity.Rates;
    M = cons.Velocity.MassMatrix;
    f = cons.Velocity.ForcingVector;
    fv = M*xd - f;
end