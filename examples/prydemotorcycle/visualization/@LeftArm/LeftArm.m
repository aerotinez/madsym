classdef LeftArm < MotoBody
    methods (Access = public)
        function obj = LeftArm(varargin)
            obj@MotoBody(varargin{:});
            setPose(obj,[0,0,0,0]);
        end
    end
end