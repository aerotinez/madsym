classdef Driver < MotoBody
    methods (Access = public)
        function obj = Driver(varargin)
            obj@MotoBody(varargin{:});
            setPose(obj,[0,0,0,0]);
        end
    end
end