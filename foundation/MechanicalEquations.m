classdef (Abstract) MechanicalEquations 
properties (GetAccess = public, SetAccess = protected)
    q sym;
    u sym;
    v sym = sym.empty(0,1);
    qd sym;
    ud sym;
    vd (:,1) sym = sym.empty(0,1);
    MassMatrix sym;
    ForcingVector sym;
    Linearized LinearizedEquations;
end
end