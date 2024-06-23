function simplify(obj)
    arguments
        obj (1,1) MotionEquations;
    end
    f = @(x)simplify(expand(x));
    obj.MassMatrix = f(obj.MassMatrix);
    obj.ForcingVector = f(obj.ForcingVector);
end