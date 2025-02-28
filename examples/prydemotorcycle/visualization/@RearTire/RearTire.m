classdef RearTire < MotoBody
    methods (Access = public)
        function obj = RearTire(varargin)
            obj@MotoBody(varargin{:});
            setPose(obj,[0,0,0,0]);
        end
    end
end