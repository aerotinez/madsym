classdef BikeBase < MotoBody
    methods (Access = public)
        function obj = BikeBase(varargin)
            obj@MotoBody(varargin{:});
            setPose(obj,[0,0,0,0]);
        end
    end
end