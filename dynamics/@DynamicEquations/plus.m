function eom = plus(eoma,eomb)
    arguments
        eoma (1,1) DynamicEquations;
        eomb (1,1) DynamicEquations;
    end
    if ~isequal(eoma.States,eomb.States) || ~isequal(eoma.Inputs,eomb.Inputs)
        error("Cannot add motion equations with different states or inputs.");
    end
    if numel(eoma.ForcingVector) ~= numel(eomb.ForcingVector)
        error("Cannot add motion equations with different dimensions.");
    end
    x = eoma.States;
    M = eoma.MassMatrix + eomb.MassMatrix;
    f0 = eoma.f0 + eomb.f0;
    f1 = eoma.f1 + eomb.f1;
    f2 = eoma.f2 + eomb.f2;
    F = eoma.Inputs;
    eom = DynamicEquations(x,M,f0,f1,f2,F);
end