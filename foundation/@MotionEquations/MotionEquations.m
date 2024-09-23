classdef MotionEquations
    properties (GetAccess = public, SetAccess = protected)
        States;
        Inputs;
        MassMatrix;
        ForcingVector;
    end
    methods (Access = public)
        function obj = MotionEquations(states,mass_matrix,forcing_vector,inputs)
            arguments
                states (:,1) DynamicVariable;
                mass_matrix sym {mustBeNonempty};
                forcing_vector (:,1) sym {mustBeNonempty};
                inputs (:,1) DynamicVariable = DynamicVariable(0,1);
            end
            obj.States = states;
            obj.MassMatrix = mass_matrix;
            obj.ForcingVector = forcing_vector;
            obj.Inputs = inputs;
        end 
    end 
end