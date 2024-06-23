function eomdl = linearizeBodyDynamicsParallel(obj)
    arguments
        obj (1,1) GibbsAppell;
    end
    x = obj.LinearizationStates;
    N = numel(obj.BodyDynamics);
    dyn = cell(N,1);
    b = obj.BodyDynamics;
    trim_point = obj.Trim;
    parfor k = 1:N
        dyn{k} = linearize(b(k),x,trim_point);
    end
    eomdl = [dyn{:}];
end