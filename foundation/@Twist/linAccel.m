function pdd = linAccel(obj)
    arguments
        obj (1,1) Twist
    end
    R = obj.Pose.ReferenceFrame.dcm();
    w = obj.angVel();
    v = obj.transVel();
    vd = obj.transAccel();
    pdd = simplify(expand(R*vec2skew(w)*v + R*vd));
end