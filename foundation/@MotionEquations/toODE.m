function eom = toODE(obj)
    arguments
        obj (1,1) MotionEquations;
    end
    x = obj.States;
    u = obj.Inputs;
    M = eye(numel(x),"sym");
    f = syminv(obj.MassMatrix)*obj.ForcingVector;
    eom = MotionEquations(x,M,f,u);
end