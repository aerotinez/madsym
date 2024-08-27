function V = vector(obj,pose)
    arguments
        obj (1,1) Twist;
        pose (1,1) Pose = obj.Pose;
    end
    Ad = eye(6,"sym");
    if pose ~= obj.Pose 
        Ad = simplify(expand(pose.inv().adjoint()*obj.Pose.adjoint()));
    end
    V = simplify(expand(Ad*obj.Vector)); 
end