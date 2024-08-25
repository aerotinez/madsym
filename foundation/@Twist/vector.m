function V = vector(obj,pose)
    arguments
        obj (1,1) Twist;
        pose (1,1) Pose = Pose();
    end
    w = obj.angVel(pose.ReferenceFrame);
    v = obj.transVel(pose);

    V = [
        w;
        v
        ];
end