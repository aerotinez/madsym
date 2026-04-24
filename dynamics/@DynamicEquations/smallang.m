function out = smallang(obj,x)
    arguments
        obj (:,1) DynamicEquations
        x (:,1) sym
    end
    out = obj;
    out.MassMatrix = smallang(obj.MassMatrix,x);
    out.f0 = smallang(obj.f0,x);
    out.f1 = smallang(obj.f1,x);
    out.f2 = smallang(obj.f2,x);
    out.ForcingVector = out.f0 + out.f1 + out.f2;
end