classdef GibbsAppell < handle
    properties (GetAccess = public, SetAccess = private)
        States;
        Inputs;
        Constraints;
        Equations;
        Trim;
        LinearizedEquations;  
        Kinematics;
        BodyDynamics;
        Dynamics;
        Auxiliary;
        LinearizationStates; 
    end
    methods (Access = public)
        function obj = GibbsAppell(states,constraints,inputs)
            arguments
                states (1,1) StateVector;
                constraints (1,1) Constraints = Constraints();
                inputs (:,1) sym = sym.empty(0,1);
            end
            obj.States = states;

            obj.LinearizationStates = [
                obj.States.Coordinates.All;
                obj.States.Speeds.Independent;
                obj.States.Auxiliary
            ];

            obj.Constraints = constraints;
            obj.validateConstraints();
            obj.Inputs = inputs;
        end 
    end 
end