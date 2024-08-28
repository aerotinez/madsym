function eoml = linearize(obj,trim_point)
    arguments
        obj (1,1) MotionEquations;
        trim_point (1,1) TrimPoint = TrimPoint.empty();
    end

    f0 = obj.MassMatrix*obj.Rates;
    f1 = -obj.ForcingVector;

    M = subsTrim(obj.MassMatrix,trim_point);

    Jf0 = subsTrim(jacobian(f0,obj.States),trim_point);
    Jf1 = subsTrim(jacobian(f1,obj.States),trim_point);

    G = -subsTrim(jacobian(f1,obj.Inputs),trim_point);
    H = -(Jf0 + Jf1);

    eoml = LinearizedMotionEquations(obj.States,M,H,obj.Inputs,G);
end