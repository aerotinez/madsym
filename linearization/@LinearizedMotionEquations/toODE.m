function eoml = toODE(obj)
    arguments
        obj (1,1) LinearizedMotionEquations;
    end
    x = obj.States;
    P = x.Pind;
    M = obj.MassMatrix;
    H = obj.ForcingMatrix;
    G = obj.InputMatrix;
    u = obj.Inputs;
    Minv = syminv(M);
    A = P.'*Minv*H;
    B = P.'*Minv*G;
    x0 = x.Pind.'*x.Trim;
    xd0 = x.Pind.'*x.TrimRate;
    x = GeneralizedCoordinates(x.Independent,[],x0,xd0);
    eoml = LinearizedMotionEquations(x,eye(size(A)),A,B,u);
end