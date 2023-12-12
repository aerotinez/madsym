function [C1,C2] = dependentVelocityProjection(coordinates,kinematics,constraints)
    arguments
        coordinates (1,1) GeneralizedCoordinates;
        kinematics (1,1) KinematicEquations;
        constraints (1,1) Constraints;
    end
    X = coordinates;
    K = kinematics;
    P = PermutationMatrices(X);
    nhc = simplify(expand(subs(constraints.Nonholonomic,X.qd,K.ForcingVector)));
    Jq = jacobian(nhc,X.q);
    Ju = jacobian(nhc,X.u);
    C1 = -P.Pu_dep*syminv(Ju*P.Pu_dep)*Jq;
    C2 = (eye(X.n) - P.Pu_dep*syminv(Ju*P.Pu_dep)*Jq)*P.Pq_ind;
end