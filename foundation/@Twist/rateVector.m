function Vd = rateVector(obj,pose)
    arguments
        obj (1,1) Twist
        pose (1,1) Pose = Pose();
    end
    wd = obj.angAccel(pose.ReferenceFrame);
    vd = obj.transAccel(pose);

    Vd = [
        wd;
        vd
    ];
end