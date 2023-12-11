classdef (Abstract) KinematicsStrategy
    methods (Access = public)
        kinematic_equations = kinematicEquations(obj)
    end
end