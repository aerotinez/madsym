function simplify(obj)
    arguments
        obj (1,1) DynamicEquations
    end
    f = @(x)simplify(expand(x));
    obj.MassMatrix = f(obj.MassMatrix);
    obj.f0 = f(obj.f0);
    obj.f1 = f(obj.f1);
    obj.f2 = f(obj.f2);
    obj.ForcingVector = f(obj.f0 + obj.f1 + obj.f2);
end
