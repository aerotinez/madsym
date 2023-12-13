classdef MotionEquations
    properties (GetAccess = public, SetAccess = private)
        States sym;
        Rates sym;
        Inputs sym = sym.empty(0,1);
        MassMatrix sym;
        ForcingVector sym;
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
            obj.validateStates();
            obj.Rates = diff(states);
            obj.MassMatrix = mass_matrix;
            obj.validateMassMatrix();
            obj.ForcingVector = forcing_vector;
            obj.validateForcingVector();
            obj.Inputs = inputs;
            obj.validateInputs();
        end 
        function eoml = linearize(obj,lin_states)
            if ~all(isDynamicVariable(lin_states))
                error('Linearization states must be dynamic variables.')
            end
            lin_rates = diff(lin_states);
            f0 = obj.MassMatrix*obj.Rates;
            f1 = -obj.ForcingVector;
            M = jacobian(f0,lin_rates);
            H = -jacobian(f0 + f1,lin_states);
            G = -jacobian(f1,obj.Inputs);
            eoml = LinearizedMotionEquations(lin_states,M,H,obj.Inputs,G);
        end
    end
    methods (Access = private)
        function validateStates(obj)
            if ~all(isDynamicVariable(obj.States))
                error('States must be dynamic variables.')
            end
        end
        function validateInputs(obj)
            if isempty(obj.Inputs)
                return
            end
            if ~all(isDynamicVariable(obj.Inputs))
                error('Inputs must be dynamic variables.')
            end
        end 
        function validateMassMatrix(obj)
            n = numel(obj.States);
            if ~isequal(size(obj.MassMatrix,1),size(obj.MassMatrix,2),n)
                msga = "Mass matrix must have as many rows and columns ";
                msgb = "as there are states.";
                error(msga + msgb);
            end
        end
        function validateForcingVector(obj)
            n = numel(obj.States);
            if ~isequal(size(obj.ForcingVector,1),n)
                msga = "Forcing vector must have as many rows as there ";
                msgb = "are states.";
                error(msga + msgb);
            end
        end
    end
end