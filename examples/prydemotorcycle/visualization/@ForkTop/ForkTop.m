classdef ForkTop < MotoBody
    methods (Access = public)
        function obj = ForkTop(varargin)
            obj@MotoBody(varargin{:});
            setPose(obj,[0,0,0,0]);
        end
    end
end