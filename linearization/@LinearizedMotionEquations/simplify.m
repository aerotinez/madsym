function eoml = simplify(obj)
    arguments
        obj (1,1) LinearizedMotionEquations;
    end
    M = simplify(expand(obj.MassMatrix));
    H = simplify(expand(obj.ForcingMatrix));
    G = simplify(expand(obj.InputMatrix));
    x = obj.States;
    u = obj.Inputs;
    eoml = LinearizedMotionEquations(x,M,H,G,u);
end