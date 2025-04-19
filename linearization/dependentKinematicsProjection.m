function [Rkq,Rkqd] = dependentKinematicsProjection(q,eomk,eomc)
    arguments
        q (:,1) DynamicVariable;
        eomk (1,1) KinematicEquations;
        eomc (1,1) ConstraintEquations;
    end
    Pqi = permMatInd(q).';
    Pqd = permMatDep(q).';

    fv = eomc.Jacobian*syminv(eomk.Jacobian)*eomc.States.rate;
    fv = simplify(expand(fv));

    Jq = jacobian(fv,q.state);
    Jqd = jacobian(fv,q.rate);
    
    D = -Pqd*syminv(Jqd*Pqd);

    Rcq = dependentCoordinateProjection(q,eomc);
    Rkq = D*Jq*Rcq;

    I = eye(numel(q));
    Rkqd = (I + D*Jqd)*Pqi;
end