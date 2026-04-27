function eom = plus(eoma,eomb)
    arguments
        eoma (1,1) DynamicEquations
        eomb (1,1) DynamicEquations
    end

    if any(eoma.States ~= eomb.States) || any(eoma.Inputs ~= eomb.Inputs)
        error("Cannot add equations with different states or inputs.");
    end

    if numel(eoma.ForcingVector) ~= numel(eomb.ForcingVector)
        error("Cannot add equations with different dimensions.");
    end

    if ~isequal(eoma.ConstraintJacobian,eomb.ConstraintJacobian)
        error("Cannot add equations with different constraint Jacobians.");
    end

    eom = eoma;
    eom.MassMatrix = eoma.MassMatrix + eomb.MassMatrix;
    eom.ForcingVector = eoma.ForcingVector + eomb.ForcingVector;

    eom.SpatialInertia = blkdiag(eoma.SpatialInertia, eomb.SpatialInertia);

    eom.Jacobian = [
        eoma.Jacobian;
        eomb.Jacobian
        ];

    eom.JacobianRate = [
        eoma.JacobianRate; 
        eomb.JacobianRate];

    eom.ActiveForces = [
        eoma.ActiveForces;
        eomb.ActiveForces
        ];

    eom.ConstraintJacobian = eoma.ConstraintJacobian;
end