classdef MotionEquations
    properties (GetAccess = public, SetAccess = protected)
        States;
        Rates;
        Inputs;
        MassMatrix;
        ForcingVector;
    end
    methods (Access = public)
        function obj = MotionEquations(states,mass_matrix,forcing_vector,inputs)
            arguments
                states (:,1) sym {mustBeNonempty};
                mass_matrix sym {mustBeNonempty};
                forcing_vector (:,1) sym {mustBeNonempty};
                inputs sym = sym.empty(0,1);
            end
            obj.States = states;
            obj.Rates = diff(states);
            obj.MassMatrix = mass_matrix;
            obj.ForcingVector = forcing_vector;
            obj.Inputs = inputs;
        end 
    end 
end