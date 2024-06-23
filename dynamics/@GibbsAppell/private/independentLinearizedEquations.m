function independentLinearizedEquations(obj)
    arguments
        obj (1,1) GibbsAppell;
    end
    P = PermutationMatrices(obj.States);
    C = dependentCoordinateProjection(obj.States,obj.Constraints);
    C = subsTrim(C,obj.Trim);

    Hk = obj.LinearizedEquations.ForcingMatrix(:,1:obj.States.n);
    Hdv = obj.LinearizedEquations.ForcingMatrix(:,obj.States.n + 1:end);

    x = obj.LinearizationStates;
    P = blkdiag(P.q_ind,eye(obj.States.k + obj.States.p,'sym'));
    M = obj.LinearizedEquations.MassMatrix;
    H = [Hk*C,Hdv];
    F = obj.Inputs;
    G = obj.LinearizedEquations.InputMatrix;

    obj.LinearizedEquations = LinearizedMotionEquations(x,M,H,F,G,P);
end