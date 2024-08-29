function eoml = linearize(obj,trim_point)
    arguments
        obj (1,1) DynamicEquations;
        trim_point (1,1) TrimPoint = TrimPoint(obj.StateVector,obj.Inputs);
    end
    t = sym('t');

    x = [
        trim_point.q;
        trim_point.u;
        ];
        
    xd = diff(x,t);

    u = trim_point.F;

    fM = obj.MassMatrix*obj.Rates;
    M = subsTrim(jacobian(fM,xd),trim_point); 
    
    f = -obj.ForcingVector;
    f0 = -obj.f0;
    f1 = -obj.f1;
    f2 = -obj.f2;

    JfM = subsTrim(jacobian(fM,x),trim_point);
    Jf0 = subsTrim(jacobian(f0,x),trim_point);
    Jf1 = subsTrim(jacobian(f1,x),trim_point);
    Jf2 = subsTrim(jacobian(f2,x),trim_point);

    G = -subsTrim(jacobian(f,u),trim_point);
    H = -(JfM + Jf0 + Jf1 + Jf2);

    eoml = LinearizedMotionEquations(x,M,H,G,u);
end