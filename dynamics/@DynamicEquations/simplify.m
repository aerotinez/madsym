function eomd = simplify(obj)
    arguments
        obj (1,1) DynamicEquations
    end

    eomd = obj;

    eomd.SpatialInertia = simplify(expand(obj.SpatialInertia));
    eomd.Jacobian = simplify(expand(obj.Jacobian));
    eomd.JacobianRate = simplify(expand(obj.JacobianRate));
    eomd.ActiveForces = simplify(expand(obj.ActiveForces));
    eomd.ConstraintJacobian = simplify(expand(obj.ConstraintJacobian));

    eomd.MassMatrix = eomd.massMatrix();
    eomd.ForcingVector = eomd.forcingVector();
end