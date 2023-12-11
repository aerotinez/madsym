classdef (Abstract) EquationsOfMotion 
properties (GetAccess = public, SetAccess = protected)
    MassMatrix sym;
    ForcingVector sym;
    Linearized LinearizedEquations;
end
end