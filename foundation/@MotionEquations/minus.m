function eom = minus(eoma,eomb)
    arguments
        eoma (1,1) MotionEquations;
        eomb (1,1) MotionEquations;
    end
    if any(eoma.States ~= eomb.States) || any(eoma.Inputs ~= eomb.Inputs)
        error("Cannot subtract equations with different states or inputs.");
    end
    if numel(eoma.ForcingVector) ~= numel(eomb.ForcingVector)
        error("Cannot substract equations with different dimensions.");
    end
    x = eoma.States;
    Ma = eoma.MassMatrix;
    Mb = eomb.MassMatrix;
    fa = eoma.ForcingVector;
    fb = eomb.ForcingVector;
    F = eoma.Inputs;
    eom = MotionEquations(x,Ma - Mb,fa - fb,F);
end