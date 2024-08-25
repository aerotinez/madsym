function adV = adjoint(obj,pose)
    arguments
        obj (1,1) Twist;
        pose (1,1) Pose = Pose();
    end
    wm = vec2skew(obj.angVel(pose.ReferenceFrame));
    vm = vec2skew(obj.transVel(pose));

    adV = [
        wm,zeros(3,3);
        vm,wm
        ];
end