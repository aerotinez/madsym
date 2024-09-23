function eomd = simplify(obj)
    arguments
        obj (1,1) DynamicEquations
    end
    u = obj.States;
    F = obj.Inputs;
    M = simplify(expand(obj.MassMatrix));
    f0 = simplify(expand(obj.f0));
    f1 = simplify(expand(obj.f1));
    f2 = simplify(expand(obj.f2));
    eomd = DynamicEquations(u,M,f0,f1,f2,F);
end
