function out = smallang(obj,x)
    arguments
        obj (:,1) KinematicEquations
        x (:,1) sym
    end
    out = obj;
    out.Jacobian = smallang(obj.Jacobian,x);
    out.JacobianRate = smallang(obj.JacobianRate,x);
    if obj.IsTrimmed
        out.ForcingVector = out.Jacobian*[obj.Inputs.TrimState].';
    else
        out.ForcingVector = out.Jacobian*state(obj.Inputs);
    end
end