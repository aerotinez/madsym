function out = smallang(obj,x)
    arguments
        obj (:,1) MotionEquations
        x (:,1) sym;
    end
    out = obj;
    out.MassMatrix = smallang(obj.MassMatrix,x);
    out.ForcingVector = smallang(obj.ForcingVector,x);
end