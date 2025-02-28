classdef SwingArm < MotoBody
    methods (Access = public)
        function obj = SwingArm(varargin)
            obj@MotoBody(varargin{:});
            setPose(obj,[0,0,0,0]);
        end
    end
end