function C = dependentCoordinateProjection(coordinates,constraints)
    arguments
        coordinates (1,1) GeneralizedCoordinates;
        constraints (1,1) Constraints;
    end
    X = coordinates;
    P = PermutationMatrices(X);
    hc = constraints.Holonomic;
    J = simplify(expand(jacobian(hc,X.q)));
    C = (eye(X.n) - P.Pq_dep*syminv(J*P.Pq_dep)*J)*P.Pq_ind;
end