function eomd = toODE(obj)
    arguments
        obj (1,1) DynamicEquations
    end
    u = obj.States;
    F = obj.Inputs;
    Minv = syminv(obj.MassMatrix);
    M = eye(size(obj.MassMatrix),"sym");
    f0 = Minv*(obj.f0);
    f1 = Minv*(obj.f1);
    f2 = Minv*(obj.f2);
    eomd = DynamicEquations(u,M,f0,f1,f2,F);
end