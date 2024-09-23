function eoml = plus(eoma,eomb)
    arguments
        eoma (1,1) LinearizedMotionEquations
        eomb (1,1) LinearizedMotionEquations
    end

    if any(eoma.States ~= eomb.States) || any(eoma.Inputs ~= eomb.Inputs)
        msga = "States and inputs must be the same for both "; 
        msgb = "LinearizedMotionEquations objects";
        error(msga + msgb);
    end

    M = eoma.MassMatrix + eomb.MassMatrix;
    H = eoma.ForcingMatrix + eomb.ForcingMatrix;
    G = eoma.InputMatrix + eomb.InputMatrix;

    eoml = LinearizedMotionEquations(eoma.States,M,H,G,eoma.Inputs);
end