function out = subsParams(obj,ovals,nvals)
    arguments
        obj (1,1) NewtonEulerEquations;
        ovals (:,1) sym;
        nvals (:,1) sym;
    end
    out = obj;
    f = @(x)subs(x,ovals,nvals);
    out.Pose = f(obj.Pose);
    out.SpatialInertia = f(obj.SpatialInertia);
    out.Twist = f(obj.Twist);
    out.TwistRate = f(obj.TwistRate);
    out.ActiveForces = f(obj.ActiveForces);
end