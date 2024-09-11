function v = transVel(obj,pose)
    arguments
        obj (1,1) Twist;
        pose (1,1) Pose = obj.Pose;
    end
    V = vector(obj,pose);
    v = V(4:6);
end