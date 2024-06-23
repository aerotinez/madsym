classdef GeneralizedCoordinates
    properties (GetAccess = public, SetAccess = private)
        All; 
        Independent;
        Dependent;
    end
    methods (Access = public)
        function obj = GeneralizedCoordinates(all,dependent)
            arguments
                all (:,1) sym {mustBeNonempty};
                dependent (:,1) sym = sym.empty(0,1);
            end
            obj.All = all;
            validateCoordinates(obj);
            obj.Dependent = dependent;
            obj.Independent = obj.All(~has(obj.All,obj.Dependent)); 
        end
    end 
end