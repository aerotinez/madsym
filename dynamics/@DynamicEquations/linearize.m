function eoml = linearize(obj,x,u)
    arguments
        obj (1,1) DynamicEquations;
        x (:,1) DynamicVariable = obj.States;
        u (:,1) DynamicVariable = obj.Inputs;
    end
    
    vars = [
        x;
        u
        ];

    fM = obj.MassMatrix*obj.States.rate;
    M = subsTrim(jacobian(fM,x.rate),vars);
    
    f = -obj.ForcingVector;
    f0 = -obj.f0;
    f1 = -obj.f1;
    f2 = -obj.f2;

    JfM = subsTrim(jacobian(fM,x.state),vars);
    Jf0 = subsTrim(jacobian(f0,x.state),vars);
    Jf1 = subsTrim(jacobian(f1,x.state),vars);
    Jf2 = subsTrim(jacobian(f2,x.state),vars);

    G = -subsTrim(jacobian(f,u.state),vars);
    H = -(JfM + Jf0 + Jf1 + Jf2);

    eoml = LinearizedMotionEquations(x,M,H,G,u);
end