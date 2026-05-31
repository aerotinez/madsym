function out = smallang(obj,x)
    arguments
        obj (1,1) NewtonEulerEquations;
        x (:,1) sym;
    end
    out = obj;
    out.SpatialInertia = smallang(obj.SpatialInertia,x);
    out.Twist = smallang(obj.Twist,x);
    out.TwistRate = smallang(obj.TwistRate,x);
    out.ActiveForces = smallang(obj.ActiveForces,x);
end