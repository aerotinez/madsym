function out = trim(obj)
    arguments
        obj (:,1) MotionEquations
    end
    out = obj;

    z = [
        obj.States;
        obj.Inputs
        ];

    out.MassMatrix = subsTrim(obj.MassMatrix,z);
    out.ForcingVector = subsTrim(obj.ForcingVector,z);
    out.IsTrimmed = true;
end