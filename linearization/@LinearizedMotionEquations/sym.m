function eqns = sym(obj)
    arguments
        obj (1,1) LinearizedMotionEquations;
    end
    xd = obj.States.rate();
    x = obj.States.state();
    u = obj.Inputs.state();

    M = obj.MassMatrix;
    H = obj.ForcingMatrix;
    G = obj.InputMatrix;

    eqns = M*xd - H*x;

    if ~isempty(u)
        eqns = eqns - G*u;
    end
end