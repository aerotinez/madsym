classdef DynamicEquations
properties (GetAccess = public, SetAccess = private)
    MassMatrix sym;
    ForcingVector sym;
    Linearized LinearizedDynamicEquations;
end
methods (Access = public)
function obj = DynamicEquations(method,kinematic_equations,bodies)
    arguments
        method function_handle;
        kinematic_equations KinematicEquations;
        bodies Body;
    end
    eomd = method(kinematic_equations,bodies);
    obj.MassMatrix = eomd.MassMatrix;
    obj.ForcingVector = eomd.ForcingVector;

end
end
end