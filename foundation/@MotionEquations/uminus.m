function eom = uminus(obj)
    arguments
        obj (1,1) MotionEquations;
    end
    x = obj.States;
    M = -obj.MassMatrix;
    f = -obj.ForcingVector;
    u = obj.Inputs;
    eom = MotionEquations(x,M,f,u);
end
