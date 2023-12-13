function C = dependentCoordinateProjection(states,constraints)
    arguments
        states (1,1) StateVector;
        constraints (1,1) Constraints;
    end
    P = PermutationMatrices(states);
    hc = constraints.Holonomic;
    q = states.Coordinates.All;
    J = simplify(expand(jacobian(hc,q)));
    C = (eye(numel(q)) - P.q_dep*syminv(J*P.q_dep)*J)*P.q_ind;
end