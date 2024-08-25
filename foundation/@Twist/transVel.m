function v = transVel(obj,pose)
    arguments
        obj (1,1) Twist;
        pose (1,1) Pose = Pose();
    end
    vs = obj.TranslationalVelocity;
    ws = obj.AngularVelocity;
    R = pose.ReferenceFrame.dcm();
    dm = vec2skew(pose.Position.posFrom());
    v_eul = simplify(expand(-R.'*dm*ws));
    v_lin = simplify(expand(R.'*vs));
    v = simplify(expand(v_eul + v_lin));
end