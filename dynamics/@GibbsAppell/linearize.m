function linearize(obj)
    arguments
        obj (1,1) GibbsAppell;
    end
    x = obj.LinearizationStates;
    eomkl = obj.Kinematics.linearize(x,obj.Trim);
    eomdl = obj.linearizeBodyDynamics();

    M = [
        eomkl.MassMatrix;
        eomdl.MassMatrix;
        ];

    H = [
        eomkl.ForcingMatrix;
        eomdl.ForcingMatrix;
        ];

    F = obj.Inputs;

    G = [
        eomkl.InputMatrix;
        eomdl.InputMatrix;
        ];
            
    if ~isempty(obj.Auxiliary)
        eomvl = obj.Auxiliary.linearize(x,obj.Trim);
        M = [M;eomvl.MassMatrix];
        H = [H;eomvl.ForcingMatrix];
        G = [G;eomvl.InputMatrix];
    end

    obj.LinearizedEquations = LinearizedMotionEquations(x,M,H,F,G);
    obj.independentLinearizedEquations();
end