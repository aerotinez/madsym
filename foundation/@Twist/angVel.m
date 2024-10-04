function w = angVel(obj,frame)
    arguments
        obj (1,1) Twist;
        frame (1,1) Frame = obj.Pose.ReferenceFrame;
    end
    R = eye(3,"sym");
    if frame ~= obj.Pose.ReferenceFrame
        R = simplify(expand(frame.dcm().'*obj.Pose.ReferenceFrame.dcm()));
    end
    w = simplify(expand(R*obj.Vector(1:3)));
end