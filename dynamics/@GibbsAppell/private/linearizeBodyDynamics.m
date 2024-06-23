function eomdl = linearizeBodyDynamics(obj)
    arguments
        obj (1,1) GibbsAppell;
    end

    x = obj.LinearizationStates;

    if isempty(gcp("nocreate")) || numel(obj.BodyDynamics) < 2
        fl = @(b)b.linearize(x,obj.Trim);
        eomdl_bodies = arrayfun(fl,obj.BodyDynamics);
    else
        eomdl_bodies = linearizeBodyDynamicsParallel(obj);
    end

    fB = @(f)arrayfun(f,eomdl_bodies,'uniform',0);
    sum3 = @(f)sum(cell2sym(reshape(fB(f),1,1,[])),3);

    M = sum3(@(b)b.MassMatrix);
    H = sum3(@(b)b.ForcingMatrix);
    F = obj.Inputs;
    G = sum3(@(b)b.InputMatrix);

    eomdl = LinearizedMotionEquations(x,M,H,F,G);
end