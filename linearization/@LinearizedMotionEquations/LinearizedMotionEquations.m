classdef LinearizedMotionEquations < handle 
    properties (GetAccess = public, SetAccess = private)
        States;
        Rates;
        Inputs;
        MassMatrix;
        ForcingMatrix;
        InputMatrix;
        PermutationMatrix;
    end
    methods (Access = public)
        function obj = LinearizedMotionEquations( ...
            states, ...
            mass_matrix, ...
            forcing_matrix, ...
            inputs, ...
            input_matrix, ...
            permutation_matrix ...
            )
            arguments
                states sym;
                mass_matrix sym;
                forcing_matrix sym;
                inputs sym = sym.empty(0,1);
                input_matrix sym = sym.empty(numel(states),0);
                permutation_matrix sym = sym.empty(0,0);
            end
            obj.States = states;
            obj.validateStates();
            obj.Rates = diff(obj.States);
            obj.MassMatrix = mass_matrix;
            obj.ForcingMatrix = forcing_matrix;
            obj.Inputs = inputs;
            obj.validateInputs();
            obj.InputMatrix = input_matrix;
            obj.PermutationMatrix = permutation_matrix;
        end 
    end 
end