classdef AppellsMethod < DynamicsMethod
    methods (Access = public)
        function obj = AppellsMethod(varargin)
            obj@DynamicsMethod(varargin{:});
        end
    end
    methods (Access = protected)
        eomd = bodyDynamics(obj,body);
    end
end