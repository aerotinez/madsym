classdef DynamicVariable < handle
    properties (GetAccess = public, SetAccess = private)
        State;
        Rate;
        IsDependent;
        TrimState;
        TrimRate;
    end
    methods (Access = public)
        function obj = DynamicVariable(str,isdep,x0,dx0dt)
            arguments
                str (1,1) string;
                isdep (1,1) logical = false; 
                x0 (1,1) sym = nan(1,1);
                dx0dt (1,1) sym = nan(1,1);
            end
            t = sym('t');
            f = symfun(str + "(t)",t);
            obj.State = f(t);
            obj.Rate = diff(obj.State,t);
            obj.IsDependent = isdep;
            obj.TrimState = obj.State;
            obj.TrimRate = obj.Rate;
            if ~isnan(x0)
                obj.TrimState = x0;
                obj.TrimRate = diff(x0,t);
            end
            if ~isnan(dx0dt)
                obj.TrimRate = dx0dt;
            end
        end
    end
end