function eoml = simplify(obj)
    arguments
        obj (1,1) LinearizedMotionEquations
    end
    f = @(x)simplify(expand(x));
    M = f(obj.MassMatrix);
    H = f(obj.ForcingMatrix);
    G = f(obj.InputMatrix);
    x = obj.States;
    u = obj.Inputs;
    eoml = LinearizedMotionEquations(x,M,H,G,u);
end