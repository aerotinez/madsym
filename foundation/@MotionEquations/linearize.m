function eoml = linearize(obj,x,u)
    arguments
        obj (1,1) MotionEquations;
        x (:,1) DynamicVariable = obj.States;
        u (:,1) DynamicVariable = obj.Inputs;
    end

    vars = [
        x;
        u
        ];
 
    f0 = obj.MassMatrix*obj.States.rate;
    f1 = -obj.ForcingVector;

    M = subsTrim(jacobian(f0,x.rate),vars); 
    Jf0 = subsTrim(jacobian(f0,x.state),vars);
    Jf1 = subsTrim(jacobian(f1,x.state),vars);

    G = -subsTrim(jacobian(f1,u.state),vars);
    H = -(Jf0 + Jf1);

    eoml = LinearizedMotionEquations(x,M,H,G,u);
end