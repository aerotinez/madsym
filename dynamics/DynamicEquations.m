classdef DynamicEquations < EquationsOfMotion
properties (GetAccess = public, SetAccess = private)
    MassMatrix sym;
    ForcingVector sym;
    Linearized LinearizedDynamicEquations;
end
methods (Access = public)
function obj = DynamicEquations(mass_matrix,forcing_vector)
    arguments 
    end
    eomd = method(kinematic_equations,bodies);
    obj.MassMatrix = eomd.MassMatrix;
    obj.ForcingVector = eomd.ForcingVector;
end
end
end