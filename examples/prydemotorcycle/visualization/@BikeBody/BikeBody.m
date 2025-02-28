classdef BikeBody < MotoBody
    methods (Access = public)
        function obj = BikeBody(varargin)
            obj@MotoBody(varargin{:});
            setPose(obj,[0,0,0,0]);
        end
    end
end