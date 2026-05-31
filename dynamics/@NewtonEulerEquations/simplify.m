function out = simplify(obj)
    arguments
        obj (1,1) NewtonEulerEquations;
    end
    out = obj;
    f = @(x)simplify(expand(x));
    out.SpatialInertia = f(obj.SpatialInertia);
    out.Twist = f(obj.Twist);
    out.TwistRate = f(obj.TwistRate);
    out.ActiveForces = f(obj.ActiveForces);
end