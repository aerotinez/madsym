function Rcq = dependentCoordinateProjection(q,eomc)
    arguments
        q (:,1) DynamicVariable;
        eomc (1,1) ConstraintEquations;
    end
    I = eye(numel(q.state));
    Pqi = permMatInd(q).';
    Pqd = permMatDep(q).';
    fc = eomc.Configuration;
    Jc = jacobian(fc,q.state);
    Rcq = (I - Pqd*syminv(Jc*Pqd)*Jc)*Pqi;
end