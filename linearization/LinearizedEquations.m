classdef LinearizedEquations
properties (GetAccess = public, SetAccess = private)
    StateVector sym;
    MassMatrix sym;
    ForcingMatrix sym;
    InputMatrix sym;
end
methods (Access = public)
function obj = LinearizedEquations(x,M,H,G)
    obj.StateVector = x;
    obj.MassMatrix = M;
    obj.ForcingMatrix = H;
    obj.InputMatrix = G;
end
end
end