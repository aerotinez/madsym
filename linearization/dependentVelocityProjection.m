function [C1,C2] = dependentVelocityProjection(q,u,constraints)
    arguments
        q (1,1) GeneralizedCoordinates;
        u (1,1) GeneralizedCoordinates;
        constraints (1,1) ConstraintEquations;
    end
    fv = constraints.Jacobian*u.All;
    Jq = jacobian(fv,q.All);
    Ju = jacobian(fv,u.All);
    Pui = u.Pind;
    Pud = u.Pdep;
    I = eye(numel(u.All));
    C1 = -Pud*syminv(Ju*Pud)*Jq;
    C2 = (I - Pud*syminv(Ju*Pud)*Ju)*Pui;
end