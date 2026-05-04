function out = smallang(obj,x)
    arguments
        obj (:,1) ConstraintEquations
        x (:,1) sym
    end
    out = obj;
    out.Configuration = smallang(obj.Configuration,x);
    out.Jacobian = smallang(obj.Jacobian,x);
    out.JacobianRate = smallang(obj.JacobianRate,x);
end