function Vm = matrix(obj,pose)
    arguments
        obj (1,1) Twist;
        pose (1,1) Pose = Pose();
    end
    w = obj.angVel(pose.ReferenceFrame);
    v = obj.transVel(pose);

    Vm = [
        vec2skew(w),v;
        zeros(1,4)
        ];
end