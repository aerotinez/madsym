classdef (Abstract) EquationsOfMotion < handle
properties (GetAccess = public, SetAccess = private)
    MassMatrix sym;
    ForcingVector sym;
    Linearized LinearizedEquations;
end
methods (Access = private) 
    linearize(obj)
end
end