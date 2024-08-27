function disp(obj)
    arguments
        obj (1,1) Pose;
    end
    disp("Reference Frame :");
    disp(obj.ReferenceFrame);
    disp("Position :");
    disp(obj.Position);
end