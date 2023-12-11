classdef LinearizedEquations
properties (GetAccess = public, SetAccess = private)
    MassMatrix sym;
    ForcingMatrix sym;
end
methods (Access = public)
function obj = LinearizedEquations(M,f)
    obj.MassMatrix = M;
    obj.ForcingMatrix = f;
end
end
end