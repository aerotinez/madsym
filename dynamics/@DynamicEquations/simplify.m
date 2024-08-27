function eomd = simplify(obj)
    arguments
        obj (1,1) DynamicEquations
    end
    f = @(x)simplify(expand(x));
    u = obj.States;
    F = obj.Inputs;
    M = f(obj.MassMatrix);
    f0 = f(obj.f0);
    f1 = f(obj.f1);
    f2 = f(obj.f2);
    eomd = DynamicEquations(u,M,f0,f1,f2,F);
end
