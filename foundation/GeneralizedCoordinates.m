classdef GeneralizedCoordinates
    properties (GetAccess = public, SetAccess = private)
        All (:,1) sym; 
        Independent (:,1) sym;
        Dependent (:,1) sym = sym.empty(0,1);
    end
    methods (Access = public)
        function obj = GeneralizedCoordinates(independent,dependent)
            arguments
                independent (:,1) sym {mustBeNonempty};
                dependent (:,1) sym = sym.empty(0,1);
            end 
            obj.Independent = independent;
            obj.Dependent = dependent;
            obj.validateIndpendentCoordinates();
            obj.validateDependentCoordinates();
            obj.All = [
                obj.Independent;
                obj.Dependent
                ];
        end
    end
    methods (Access = private)
        function validateIndpendentCoordinates(obj)
            if ~all(isDynamicVariable(obj.Independent))
                error('Independent coordinates must be dynamic variables')
            end
        end
        function validateDependentCoordinates(obj)
            if isempty(obj.Dependent)
                return
            end
            if ~all(isDynamicVariable(obj.Dependent))
                error('Dependent coordinates must be dynamic variables')
            end
        end
    end
end