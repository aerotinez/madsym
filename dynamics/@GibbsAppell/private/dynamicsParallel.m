function out = dynamicsParallel(obj,bodies)
    arguments
        obj (1,1) GibbsAppell
        bodies (:,1) Body
    end
    N = numel(bodies);
    dyn = cell(N,1);
    parfor k = 1:N
        dyn{k} = bodyDynamics(obj,bodies(k));
    end
    out = [dyn{:}];
end