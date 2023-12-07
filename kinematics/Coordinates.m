classdef Coordinates
properties (GetAccess = public, SetAccess = private)
    States (:,1) sym = sym.empty([0,1]);
    Rates (:,1) sym = sym.empty([0,1]);
end
methods (Access = public)
function obj = Coordinates(q)
    arguments
        q (:,1) sym = sym.empty([0,1]);
    end
    obj.States = q;
    obj.validate();
    obj.Rates = diff(q,sym('t'));
end
end
methods (Access = private)
function validate(obj)
    if any(~(arrayfun(@(x)has(x,sym('t')),obj.States)))
        error("Coordinates must be functions of time SYM('t')");
    end
end
end
end