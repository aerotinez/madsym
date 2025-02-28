classdef Handlebars < MotoBody
    methods (Access = public)
        function obj = Handlebars(varargin)
            obj@MotoBody(varargin{:});
            setPose(obj,[0,0,0,0]);
        end
    end
end