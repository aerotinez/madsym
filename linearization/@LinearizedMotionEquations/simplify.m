function simplify(obj)
    arguments
        obj (1,1) LinearizedMotionEquations
    end
    f = @(x)simplify(expand(x));
    obj.MassMatrix = f(obj.MassMatrix);
    obj.ForcingMatrix = f(obj.ForcingMatrix);
end