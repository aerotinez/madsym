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

    vals = [
        nonzeros(triu(x*x.',1));
        x.^2;
        x.^3;
        x.^4;
        x.^5;
        x.^6;
        x.^7;
        x.^8;
        x.^9;
        x.^10;
        x.^11;
        x.^12
        ];

    out.MassMatrix = subs(expand(out.massMatrix()),vals,0*vals);
    out.ForcingVector = subs(expand(out.forcingVector()),vals,0*vals);
end