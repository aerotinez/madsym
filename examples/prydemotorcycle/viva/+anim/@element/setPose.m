function setPose(obj,opts)
    arguments
        obj
        opts.Orientation (3,3) double = eye(3);
        opts.Position (1,3) double  = zeros(1,3);
    end
    setOrientation(obj,opts.Orientation);
    setPosition(obj,opts.Position);
end