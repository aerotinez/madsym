classdef frame < anim.element

    properties (SetAccess = protected)
        XAxis;
        YAxis;
        ZAxis;
    end

    methods (Access = public)
        R = getOrientation(obj);
        p = getPosition(obj);
        setAlpha(obj,alpha);
        setColor(obj,c);
    end

    methods (Access = protected)
        init(obj, opts)
        setOrientation(obj,R);
        setPosition(obj,p);
    end

end