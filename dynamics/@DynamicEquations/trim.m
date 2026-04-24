function out = trim(obj,z)
    arguments
        obj (:,1) DynamicEquations
        z (:,1) DynamicVariable
    end
    out = obj;
    out.MassMatrix = subsTrim(obj.MassMatrix,z);
    out.f0 = subsTrim(obj.f0,z);
    out.f1 = subsTrim(obj.f1,z);
    out.f2 = subsTrim(obj.f2,z);
    out.ForcingVector = out.f0 + out.f1 + out.f2;
    out.IsTrimmed = true;
end