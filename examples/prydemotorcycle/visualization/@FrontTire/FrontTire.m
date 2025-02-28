classdef FrontTire < MotoBody
    methods (Access = public)
        function obj = FrontTire(varargin)
            obj@MotoBody(varargin{:});
            setPose(obj,[0,0,0,0]);
        end
    end
end