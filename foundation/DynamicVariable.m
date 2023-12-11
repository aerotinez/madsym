classdef DynamicVariable
    properties (GetAccess = public, SetAccess = private)
        State;
        Rate;
    end
    properties (Access = private)
        time = sym('t');
    end
    methods (Access = public)
        function obj = DynamicVariable(name)
            arguments
                name (1,:) {mustBeText};
            end
            obj.State = obj.stateFun(name);
            obj.Rate = diff(obj.State,obj.time); 
        end
    end
    methods (Access = private)
        function q = stateFun(obj,name)
            f = symfun(sprintf("%s(t)",name),obj.time);
            q = f(obj.time);
        end
    end
end