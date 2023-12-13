classdef GeneralizedCoordinates
    properties (GetAccess = public, SetAccess = private)
        All (:,1) sym; 
        Independent (:,1) sym;
        Dependent (:,1) sym = sym.empty(0,1);
    end
    methods (Access = public)
        function obj = GeneralizedCoordinates(all,dependent)
            arguments
                all (:,1) sym {mustBeNonempty};
                dependent (:,1) sym = sym.empty(0,1);
            end
            obj.All = all;
            obj.validateCoordinates();
            obj.Dependent = dependent;
            obj.Independent = obj.All(~has(obj.All,obj.Dependent)); 
        end
    end
    methods (Access = private)
        function validateCoordinates(obj)
            if ~all(isDynamicVariable(obj.All))
                error('Coordinates must be dynamic variables')
            end
        end
        function validateDependentCoordinates(obj)
            if isempty(obj.Dependent)
                return
            end
            if ~all(ismember(obj.Dependent,obj.All)) 
                error('Dependent coordinates must be member of all coordinates')
            end
        end
    end
end