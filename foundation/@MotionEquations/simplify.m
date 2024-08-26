function eom = simplify(obj)
    arguments
        obj (1,1) MotionEquations;
    end
    g = @(x)simplify(expand(x));
    M = g(obj.MassMatrix);
    f = g(obj.ForcingVector);
    eom = MotionEquations(obj.States,M,f,obj.Inputs);
end