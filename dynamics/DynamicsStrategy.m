classdef (Abstract) DynamicsStrategy
    methods (Access = public)
        dynamic_equations = dynamicEquations(obj)
    end
end