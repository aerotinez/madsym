function eom = subsParams(obj,ovals,nvals)
    arguments
        obj (1,1) DynamicEquations
        ovals sym
        nvals sym
    end

    eom = obj;
    f = @(x) subs(x,ovals,nvals);

    eom.SpatialInertia = f(obj.SpatialInertia);
    eom.Jacobian = f(obj.Jacobian);
    eom.JacobianRate = f(obj.JacobianRate);
    eom.ActiveForces = f(obj.ActiveForces);
    eom.ConstraintJacobian = f(obj.ConstraintJacobian);

    eom.MassMatrix = eom.massMatrix();
    eom.ForcingVector = eom.forcingVector();
end