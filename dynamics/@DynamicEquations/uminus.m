function eom = uminus(obj)
    arguments
        obj (:,1) DynamicEquations;
    end
    u = obj.States;
    M = -obj.MassMatrix;
    f0 = -obj.f0;
    f1 = -obj.f1;
    f2 = -obj.f2;
    F = obj.Inputs;
    eom = DynamicEquations(u,M,f0,f1,f2,F);
end