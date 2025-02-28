classdef FrontFender < MotoBody
    methods (Access = public)
        function obj = FrontFender(varargin)
            obj@MotoBody(varargin{:});
            setPose(obj,[0,0,0,0]);
        end
    end
end