function out = trim(obj)
    arguments
        obj (:,1) KinematicEquations
    end
    out = obj;

    z = [
        obj.States;
        obj.Inputs
        ];

    out.Jacobian = subsTrim(obj.Jacobian,z);
    out.JacobianRate = subsTrim(obj.JacobianRate,z);
    out.ForcingVector = out.Jacobian*[obj.Inputs.TrimState].';
    out.IsTrimmed = true;
end