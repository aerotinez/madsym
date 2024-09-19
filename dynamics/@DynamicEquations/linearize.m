function eoml = linearize(obj,eomk,eoma)
    arguments
        obj (1,1) DynamicEquations;
        eomk (1,1) KinematicEquations;
        eoma (:,1) MotionEquations = MotionEquations.empty(0,1);
    end
    t = sym('t');

    X = [
        eomk.States;
        obj.States;
        ];

    x = [
        diff(X.All,t);
        X.All;
        diff(obj.Inputs.All,t);
        obj.Inputs.All
        ];
        
    x0 = [
        X.TrimRate;
        X.Trim;
        obj.Inputs.TrimRate;
        obj.Inputs.Trim
        ];

    if ~isempty(eoma)
        X = eoma.States;

        x = [
            diff(X.All,t);
            X.All;
            diff(obj.Inputs.All,t);
            obj.Inputs.All
        ];

        x0 = [
            X.TrimRate;
            X.Trim;
            diff(obj.Inputs.All,t);
            obj.Inputs.All
        ];
    end

    fM = obj.MassMatrix*diff(obj.States.All,t);
    M = subs(massMatrixForm(fM,X.All),x,x0); 
    
    f = -obj.ForcingVector;
    f0 = -obj.f0;
    f1 = -obj.f1;
    f2 = -obj.f2;

    JfM = subs(jacobian(fM,X.All),x,x0);
    Jf0 = subs(jacobian(f0,X.All),x,x0);
    Jf1 = subs(jacobian(f1,X.All),x,x0);
    Jf2 = subs(jacobian(f2,X.All),x,x0);

    G = -subs(jacobian(f,obj.Inputs.All),x,x0);
    H = -(JfM + Jf0 + Jf1 + Jf2);

    eoml = LinearizedMotionEquations(X,M,H,G,obj.Inputs);
end