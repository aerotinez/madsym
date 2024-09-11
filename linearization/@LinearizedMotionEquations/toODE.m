function eoml = toODE(obj)
    arguments
        obj (1,1) LinearizedMotionEquations;
    end
    x = obj.States;
    M = obj.MassMatrix;
    H = obj.ForcingMatrix;
    G = obj.InputMatrix;
    u = obj.Inputs;
    Minv = syminv(M);
    eoml = LinearizedMotionEquations(x,eye(size(M)),Minv*H,Minv*G,u);
end