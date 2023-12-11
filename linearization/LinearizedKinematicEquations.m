classdef LinearizedKinematicEquations
properties (GetAccess = public, SetAccess = private)
    MassMatrix sym;
    ForcingMatrix sym;
end
methods (Access = public)
function obj = LinearizedKinematicEquations(q,u,f)
    n = numel(q);
    k = numel(u);
    obj.MassMatrix = [eye(n,'sym'), zeros(n,k,'sym')];
    obj.ForcingMatrix = -[jacobian(f,q),f];
end
end
end