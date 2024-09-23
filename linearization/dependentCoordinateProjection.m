function C = dependentCoordinateProjection(states,constraints)
    arguments
        states (:,1) DynamicVariable;
        constraints (1,1) ConstraintEquations;
    end
    I = eye(numel(states.state));
    Pqi = permMatInd(states).';
    Pqd = permMatDep(states).';
    fc = constraints.Configuration;
    Jc = jacobian(fc,states.state);
    C = (I - Pqd*syminv(Jc*Pqd)*Jc)*Pqi;
end