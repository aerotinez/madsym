function dynamics(obj,bodies)
    arguments
        obj (1,1) GibbsAppell;
        bodies (:,1) Body;
    end

    if isempty(gcp("nocreate")) || numel(bodies) < 2
        obj.BodyDynamics = arrayfun(@(b)obj.bodyDynamics(b),bodies);
    else
        obj.BodyDynamics = dynamicsParallel(obj,bodies);
    end
    
    u = obj.States.Speeds.Independent;
    
    fB = @(f)arrayfun(f,obj.BodyDynamics,'uniform',0);
    sum3 = @(f)sum(cell2sym(reshape(fB(f),1,1,[])),3);

    M = sum3(@(b)b.MassMatrix);
    f = sum3(@(b)b.ForcingVector);
    obj.Dynamics = MotionEquations(u,M,f,obj.Inputs);
end