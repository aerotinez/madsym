function out = smallang(obj,x)
    arguments
        obj (1,1) DynamicEquations
        x (:,1) sym
    end

    out = obj;

    out.SpatialInertia = smallang(obj.SpatialInertia,x);
    out.Jacobian = smallang(obj.Jacobian,x);
    out.JacobianRate = smallang(obj.JacobianRate,x);
    out.ActiveForces = smallang(obj.ActiveForces,x);
    out.ConstraintJacobian = smallang(obj.ConstraintJacobian,x);

    out.MassMatrix = out.massMatrix();
    out.ForcingVector = out.forcingVector();
end