function Vd = rateVector(obj,pose)
    arguments
        obj (1,1) Twist
        pose (1,1) Pose = obj.Pose;
    end
    Ad = eye(6,"sym");
    if pose ~= obj.Pose 
        Ad = simplify(expand(pose.inv().adjoint()*obj.Pose.adjoint()));
    end
    Vd = simplify(expand(Ad*obj.RateVector));
end