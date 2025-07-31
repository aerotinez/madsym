function Q = generalizedForces(obj,q)
    arguments
        obj (1,1) Body;
        q (:,1) DynamicVariable;
    end
    T = Pose(obj.ReferenceFrame,obj.MassCenter);
    Q = obj.Twist.jacobian(q).'*obj.ActiveForces.vector(T);
end