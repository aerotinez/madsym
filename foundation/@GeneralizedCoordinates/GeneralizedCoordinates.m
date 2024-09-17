classdef GeneralizedCoordinates
    properties (GetAccess = public, SetAccess = private)
        All; 
        Independent;
        Dependent;
        Trim;
        TrimRate;
    end
    properties (GetAccess = public, SetAccess = private, Hidden = true)
        P;
        Pind;
        Pdep;
    end
    methods (Access = public)
        function obj = GeneralizedCoordinates(all,dependent,trim,trim_rate)
            arguments
                all (:,1) sym {mustBeNonempty};
                dependent (:,1) sym = sym.empty(0,1);
                trim (:,1) sym = all;
                trim_rate (:,1) = diff(all,sym('t'));
            end
            obj.All = all;
            validateCoordinates(obj);
            obj.Dependent = dependent;
            obj.Independent = obj.All(~has(obj.All,obj.Dependent));
            obj.Trim = trim;
            obj.TrimRate = trim_rate;

            qbar = [
                obj.Independent;
                obj.Dependent
                ];

            obj.P = jacobian(obj.All,qbar);
            obj.Pind = obj.P(:,1:numel(obj.Independent));
            obj.Pdep = obj.P(:,(numel(obj.Independent) + 1):end);
        end
    end 
end