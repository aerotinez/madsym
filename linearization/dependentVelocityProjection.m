function [C1,C2] = dependentVelocityProjection(q,u,constraints)
    arguments
        q (:,1) DynamicVariable;
        u (:,1) DynamicVariable;
        constraints (1,1) ConstraintEquations;
    end
    fv = constraints.Jacobian*u.state;
    Jq = jacobian(fv,q.state);
    Ju = jacobian(fv,u.state);
    Pui = permMatInd(u).';
    Pud = permMatDep(u).';
    I = eye(numel(u.state));
    C1 = -Pud*syminv(Ju*Pud)*Jq;
    C2 = (I - Pud*syminv(Ju*Pud)*Ju)*Pui;
end