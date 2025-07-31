function eom = subsParams(obj,ovals,nvals)
    arguments
        obj (1,1) DynamicEquations
        ovals sym;
        nvals sym;
    end
    u = obj.States;
    F = obj.Inputs;
    f = @(x)subs(x,ovals,nvals);
    M = f(obj.MassMatrix);
    f0 = f(obj.f0);
    f1 = f(obj.f1);
    f2 = f(obj.f2);
    eom = DynamicEquations(u,M,f0,f1,f2,F);
end