function out = trim(obj)
    arguments
        obj (:,1) ConstraintEquations
    end
    out = obj;

    z = [
        obj.States;
        obj.Speeds
        ];

    out.Configuration = subsTrim(obj.Configuration,z);
    out.Jacobian = subsTrim(obj.Jacobian,z);
    out.JacobianRate = subsTrim(obj.JacobianRate,z);
    out.IsTrimmed = true;
end