classdef frontchassis < handle

    properties
        HandleBars
        ForkTop
        Fork
        FrontFender
    end

    methods (Access = public)
        setPose(obj, opts);
        setAlpha(obj, alpha);
        setColor(obj, color);
    end

end