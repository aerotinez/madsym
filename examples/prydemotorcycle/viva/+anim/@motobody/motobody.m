classdef (Abstract) motobody < anim.body
    properties (SetAccess = protected)
        Parameters
    end

    methods (Access  = public)
        setPose(obj, opts);
    end

    methods (Access = protected)
        init(obj, opts);
    end
end