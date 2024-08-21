function toODE(obj)
    arguments
        obj (1,1) MotionEquations;
    end
    obj.ForcingVector = syminv(obj.MassMatrix)*obj.ForcingVector;
    obj.MassMatrix = eye(size(obj.MassMatrix),"sym");
end