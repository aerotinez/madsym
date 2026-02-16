classdef axis < anim.element
    properties (SetAccess = protected)
        Direction
        DimensionLine
    end

    methods (Access  = public)
        R = getOrientation(obj);
        p = getPosition(obj);
        setAlpha(obj,alpha);
        setColor(obj, color);
    end

    methods (Access = protected)
        init(obj, opts);
        setOrientation(obj,R);
        setPosition(obj,p);
    end
end
