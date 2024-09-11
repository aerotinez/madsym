function vd = transAccel(obj,pose)
    arguments
        obj (1,1) Twist;
        pose (1,1) Pose = obj.Pose;
    end
    Vd = obj.rateVector(pose);
    vd = Vd(4:6);
end