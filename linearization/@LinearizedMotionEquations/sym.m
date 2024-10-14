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

    eqns = M*xd;
    if size(H,2) < numel(x)
        eqns = eqns - H*obj.States.independent().state();
    else
        eqns = eqns - H*x;
    end

    if ~isempty(u)
        eqns = eqns - G*u;
    end
end