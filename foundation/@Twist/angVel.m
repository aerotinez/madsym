function w = angVel(obj,frame)
    arguments
        obj (1,1) Twist
        frame (1,1) Frame = Frame();
    end
    ws = obj.AngularVelocity;
    R = frame.dcm();
    w = simplify(expand(R*ws));
end