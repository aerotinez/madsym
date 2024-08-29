function eoml = linearize(obj,trim_point)
    arguments
        obj (1,1) MotionEquations;
        trim_point (1,1) TrimPoint = TrimPoint.empty();
    end

    t = sym('t');

    x = [
        trim_point.q;
        trim_point.u;
        ];
        
    xd = diff(x,t);

    u = trim_point.F;

    M = subsTrim(jacobian(obj.MassMatrix*obj.Rates,xd),trim_point); 
    
    f0 = obj.MassMatrix*obj.Rates;
    f1 = -obj.ForcingVector;

    Jf0 = subsTrim(jacobian(f0,x),trim_point);
    Jf1 = subsTrim(jacobian(f1,x),trim_point);

    G = -subsTrim(jacobian(f1,u),trim_point);
    H = -(Jf0 + Jf1);

    eoml = LinearizedMotionEquations(x,M,H,G,u);
end