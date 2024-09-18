function eoml = linearize(obj)
    arguments
        obj (1,1) MotionEquations;
    end

    t = sym('t');

    x = [
        diff(obj.States.All,t);
        obj.States.All;
        diff(obj.Inputs.All,t);
        obj.Inputs.All
    ];

    x0 = [
        obj.States.TrimRate;
        obj.States.Trim;
        obj.Inputs.TrimRate;
        obj.Inputs.Trim
    ];

    M = subs(obj.MassMatrix,x,x0); 
    
    f0 = obj.MassMatrix*diff(obj.States.All,t);
    f1 = -obj.ForcingVector;

    Jf0 = subs(jacobian(f0,obj.States.All),x,x0);
    Jf1 = subs(jacobian(f1,obj.States.All),x,x0);

    G = -subs(jacobian(f1,obj.Inputs.All),x,x0);
    H = -(Jf0 + Jf1);

    eoml = LinearizedMotionEquations(obj.States,M,H,G,obj.Inputs);
end