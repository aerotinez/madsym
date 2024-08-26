function w = angVel(obj,frame)
    arguments
        obj (1,1) Twist
        frame (1,1) Frame = Frame();
    end
    w = simplify(expand(frame.dcm().'*obj.w));
end