classdef RightLeg < MotoBody
    methods (Access = public)
        function obj = RightLeg(varargin)
            obj@MotoBody(varargin{:});
            setPose(obj,[0,0,0,0]);
        end
    end
end