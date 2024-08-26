function eom = toODE(obj)
    arguments
        obj (1,1) MotionEquations;
    end
    M = eye(size(obj.MassMatrix),"sym");
    f = syminv(obj.MassMatrix)*obj.ForcingVector;
    eom = MotionEquations(obj.States,M,f,obj.Inputs);
end
