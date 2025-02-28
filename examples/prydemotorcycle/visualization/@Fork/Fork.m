classdef Fork < MotoBody
    methods (Access = public)
        function obj = Fork(varargin)
            obj@MotoBody(varargin{:});
            setPose(obj,[0,0,0,0]);
        end
    end
end