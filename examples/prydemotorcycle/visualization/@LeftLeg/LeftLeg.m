classdef LeftLeg < MotoBody
    methods (Access = public)
        function obj = LeftLeg(varargin)
            obj@MotoBody(varargin{:});
            setPose(obj,[0,0,0,0]);
        end
    end
end