function eom = plus(eoma,eomb)
    arguments
        eoma (1,1) MotionEquations;
        eomb (1,1) MotionEquations;
    end
    if eoma.States ~= eomb.States || eoma.Inputs ~= eomb.Inputs
        error("Cannot add motion equations with different states or inputs.");
    end
    if numel(eoma.ForcingVector) ~= numel(eomb.ForcingVector)
        error("Cannot add motion equations with different dimensions.");
    end
    x = eoma.States;
    Ma = eoma.MassMatrix;
    Mb = eomb.MassMatrix;
    fa = eoma.ForcingVector;
    fb = eomb.ForcingVector;
    F = eoma.Inputs;
    eom = MotionEquations(x,Ma + Mb,fa + fb,F);
end