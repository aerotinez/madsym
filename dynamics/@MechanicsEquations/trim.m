function out = trim(obj)
    arguments
        obj (:,1) MechanicsEquations
    end
    out = obj;

    z = [
        obj.States;
        obj.Inputs
        ];

    out.Constraints = trim(obj.Constraints);
    out.Kinematics = trim(obj.Kinematics);
    out.BodyDynamics = arrayfun(@(b)trim(b,z),obj.BodyDynamics);
    out.Auxiliary = trim(obj.Auxiliary);

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

    out.ForcingVector = [
        out.Kinematics.ForcingVector;
        fd;
        -out.Constraints.JacobianRate*[out.Kinematics.Inputs.TrimState].';
        out.Auxiliary.ForcingVector
        ];

    out.IsTrimmed = true;
end