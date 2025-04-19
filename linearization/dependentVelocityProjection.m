function [Rvq,Rvu] = dependentVelocityProjection(q,u,eomc)
    arguments
        q (:,1) DynamicVariable;
        u (:,1) DynamicVariable;
        eomc (1,1) ConstraintEquations;
    end
    fv = eomc.velocity;
    Jq = jacobian(fv,q.state);
    Ju = jacobian(fv,u.state);
    Pui = permMatInd(u).';
    Pud = permMatDep(u).';
    I = eye(numel(u.state));
    Rcq = dependentCoordinateProjection(q,eomc);
    D = -Pud*syminv(Ju*Pud);
    Rvq = D*Jq*Rcq;
    Rvu = (I + D*Ju)*Pui;
end