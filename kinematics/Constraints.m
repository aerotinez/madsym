classdef Constraints
properties (GetAccess = public, SetAccess = private)
    Holonomic (:,1) sym = sym.empty(0,1);
    Nonholonomic (:,1) sym = sym.empty(0,1);
end
methods (Access = public)
function obj = Constraints(holonomic,nonholonomic)
    arguments
        holonomic (:,1) sym = sym.empty(0,1);
        nonholonomic (:,1) sym = sym.empty(0,1); 
    end
    obj.Holonomic = holonomic;
    obj.Nonholonomic = [
        simplify(expand(diff(holonomic,sym('t'))));    
        nonholonomic;
        ];
end
end
end