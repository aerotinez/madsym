function out = smallang(obj,x)
    arguments
        obj (:,1) MechanicsEquations
        x (:,1) sym
    end
    out = obj;

    out.Constraints = smallang(obj.Constraints,x);
    out.Kinematics = smallang(obj.Kinematics,x);
    out.BodyDynamics = arrayfun(@(b)smallang(b,x),obj.BodyDynamics);
    out.Auxiliary = smallang(obj.Auxiliary,x);

    Mk = out.Kinematics.MassMatrix;

    Md = arrayfun(@(b)b.MassMatrix,out.BodyDynamics,'uniform',0);
    Md = [
        sum(cell2sym(reshape(Md,1,1,[])),3);
        out.Constraints.Jacobian;
        ];

    Mv = out.Auxiliary.MassMatrix;

    out.MassMatrix = blkdiag(Mk,Md,Mv);

    fd = arrayfun(@(b)b.ForcingVector,out.BodyDynamics,'uniform',0);
    fd = sum(cell2sym(reshape(fd,1,1,[])),3);

    if obj.IsTrimmed
        out.ForcingVector = [
            out.Kinematics.ForcingVector;
            fd;
            -out.Constraints.JacobianRate*[out.Kinematics.Inputs.TrimState].';
            out.Auxiliary.ForcingVector
            ];
    else
        out.ForcingVector = [
            out.Kinematics.ForcingVector;
            fd;
            -out.Constraints.JacobianRate*state(out.Kinematics.Inputs);
            out.Auxiliary.ForcingVector
            ];
    end
end