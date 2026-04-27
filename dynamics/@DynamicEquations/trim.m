function out = trim(obj,z)
    arguments
        obj (1,1) DynamicEquations
        z (:,1) DynamicVariable
    end

    out = obj;

    out.SpatialInertia = subsTrim(obj.SpatialInertia,z);
    out.Jacobian = subsTrim(obj.Jacobian,z);
    out.JacobianRate = subsTrim(obj.JacobianRate,z);
    out.ActiveForces = subsTrim(obj.ActiveForces,z);
    out.ConstraintJacobian = subsTrim(obj.ConstraintJacobian,z);
    out.IsTrimmed = true;
    out.MassMatrix = out.massMatrix();
    out.ForcingVector = out.forcingVector();
end