function out = smallang(obj,x)
    arguments
        obj (:,1) LinearizedMotionEquations
        x (:,1) sym;
    end
    out = obj;
    out.MassMatrix = smallang(obj.MassMatrix,x);
    obj.ForcingMatrix = smallang(obj.ForcingMatrix,x);
    obj.InputMatrix = smallang(obj.InputMatrix,x);
end