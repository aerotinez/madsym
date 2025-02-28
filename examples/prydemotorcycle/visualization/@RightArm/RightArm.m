classdef RightArm < MotoBody
    methods (Access = public)
        function obj = RightArm(varargin)
            obj@MotoBody(varargin{:});
            setPose(obj,[0,0,0,0]);
        end
    end
end