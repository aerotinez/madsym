function v = transAccel(obj,pose)
    arguments
        obj (1,1) Twist;
        pose (1,1) Pose = Pose();
    end
    R = pose.ReferenceFrame.dcm();
    p = pose.Position.posFrom();
    v = simplify(expand(-R.'*vec2skew(p)*obj.wd + R.'*obj.vd));
end