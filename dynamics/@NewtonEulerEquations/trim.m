function out = trim(obj,z)
    arguments
        obj (1,1) NewtonEulerEquations
        z (:,1) DynamicVariable
    end
    out = obj;
    out.Pose = subsTrim(obj.Pose,z);
    out.SpatialInertia = subsTrim(obj.SpatialInertia,z);
    out.Twist = subsTrim(obj.Twist,z);
    out.TwistRate = subsTrim(obj.TwistRate,z);
    out.ActiveForces = subsTrim(obj.ActiveForces,z);
    out.IsTrimmed = true;
end