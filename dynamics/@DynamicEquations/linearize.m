function eoml = linearize(obj,eomk)
    arguments
        obj (1,1) DynamicEquations;
        eomk (1,1) KinematicEquations;
    end
    t = sym('t');

    X = [
        eomk.States;
        obj.States;
        ];

    x = [
        diff(X.All,t);
        X.All;
        obj.Inputs.All
        ];
        
    x0 = [
        X.TrimRate;
        X.Trim;
        obj.Inputs.Trim
        ];

    fM = obj.MassMatrix*diff(obj.States.All,t);
    N = zeros(size(obj.MassMatrix,1),numel(eomk.States.All));
    M = subs([N,obj.MassMatrix],x,x0); 
    
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