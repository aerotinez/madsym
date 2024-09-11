function C = dependentCoordinateProjection(states,constraints)
    arguments
        states (1,1) GeneralizedCoordinates;
        constraints (1,1) ConstraintEquations;
    end
    I = eye(numel(states.All));
    Pqi = states.Pind;
    Pqd = states.Pdep;
    fc = constraints.Configuration;
    Jc = jacobian(fc,states.All);
    C = (I - Pqd*syminv(Jc*Pqd)*Jc)*Pqi;
end