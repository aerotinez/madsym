function dynamics(obj,bodies)
    arguments
        obj (1,1) AppellsMethod;
        bodies (:,1) Body;
    end
    obj.Bodies = bodies;
    obj.BodyDynamics = arrayfun(@obj.bodyDynamics,bodies);
    fM = @(b)b.MassMatrix(:);
    k = obj.States.k;
    u = GeneralizedCoordinates(obj.States.Speeds.Independent);
    M = reshape(sum(cell2sym(arrayfun(fM,obj.BodyDynamics,'uniform',0)),2),k,k);
    f0 = sum(cell2sym(arrayfun(@(b)b.f0,obj.BodyDynamics,'uniform',0)),2);
    f1 = sum(cell2sym(arrayfun(@(b)b.f1,obj.BodyDynamics,'uniform',0)),2);
    f2 = sum(cell2sym(arrayfun(@(b)b.f2,obj.BodyDynamics,'uniform',0)),2);
    F = obj.Inputs;
    obj.Dynamics = DynamicEquations(u,M,f0,f1,f2,F);
end