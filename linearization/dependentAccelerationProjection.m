function [Raq,Rau,Raud] = dependentAccelerationProjection(q,u,eomc)
    arguments
        q (:,1) DynamicVariable;
        u (:,1) DynamicVariable;
        eomc (1,1) ConstraintEquations;
    end
    Pui = permMatInd(u).';
    Pud = permMatDep(u).';

    fa = eomc.acceleration;

    Jq = jacobian(fa,q.state);
    Ju = jacobian(fa,u.state);
    Jud = jacobian(fa,u.rate);

    Rcq = dependentCoordinateProjection(q,eomc);
    [Rvq,Rvu] = dependentVelocityProjection(q,u,eomc);

    I = eye(numel(u.state));
    D = -Pud*syminv(Jud*Pud);
    Raq = D*(Jq*Rcq + Ju*Rvq);
    Rau = D*Ju*Rvu;
    Raud = (I + D*Jud)*Pui;
end