classdef LinearizedMotionEquations 
    properties (GetAccess = public, SetAccess = private)
        States;
        Rates;
        Inputs;
        MassMatrix;
        ForcingMatrix;
        InputMatrix;
    end
    methods (Access = public)
        function obj = LinearizedMotionEquations( ...
            states, ...
            mass_matrix, ...
            forcing_matrix, ...
            input_matrix, ...
            inputs ...
            )
            arguments
                states sym;
                mass_matrix sym;
                forcing_matrix sym;
                input_matrix sym = sym.empty(numel(states),0);
                inputs sym = sym.empty(0,1);
            end
            obj.States = states;
            obj.Rates = diff(obj.States);
            obj.MassMatrix = mass_matrix;
            obj.ForcingMatrix = forcing_matrix;
            obj.Inputs = inputs;
            obj.InputMatrix = input_matrix;
        end 
    end 
end