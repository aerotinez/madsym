classdef rearchassis < handle

    properties
        SwingArm
        Base
        Body
        Driver
        LeftArm
        LeftLeg
        RightArm
        RightLeg
    end

    methods (Access = public)
        setPose(obj, opts);
        setAlpha(obj, alpha);
        setColor(obj, color);
    end
    
end