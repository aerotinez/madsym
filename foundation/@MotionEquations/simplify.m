function eom = simplify(obj)
    arguments
        obj (1,1) MotionEquations;
    end
    M = simplify(expand(obj.MassMatrix));
    f = simplify(expand(obj.ForcingVector));
    eom = MotionEquations(obj.States,M,f,obj.Inputs);
end