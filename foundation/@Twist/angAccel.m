function wd = angAccel(obj,frame)
    arguments
        obj (1,1) Twist
        frame (1,1) Frame = obj.Pose.ReferenceFrame;
    end
    R = eye(3);
    if frame ~= obj.Pose.ReferenceFrame
        R = simplify(expand(frame.dcm().'*obj.Pose.ReferenceFrame.dcm()));
    end
    Vd = obj.rateVector();
    wd = simplify(expand(R*Vd(1:3)));
end