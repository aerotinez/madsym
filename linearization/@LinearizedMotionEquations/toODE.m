function eoml = toODE(obj)
    arguments
        obj (1,1) LinearizedMotionEquations;
    end
    x = obj.States;
    P = permMatInd(x);
    M = obj.MassMatrix;
    H = obj.ForcingMatrix;
    G = obj.InputMatrix;
    u = obj.Inputs;
    Minv = syminv(M);
    A = P*Minv*H;
    B = P*Minv*G;
    eoml = LinearizedMotionEquations(x.independent(),eye(size(A)),A,B,u);
end