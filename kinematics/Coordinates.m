classdef Coordinates
properties (GetAccess = public, SetAccess = private)
    States;
    Rates;
end
methods (Access = public)
function obj = Coordinates(q)
    arguments
        q (:,1) sym = sym.empty;
    end
    obj.States = q;
    obj.validate(q);
    obj.Rates = diff(q,sym('t'));
end
end
methods (Access = private)
function validate(obj)
    if any(~(arrayfun(@(x)has(x,sym('t')),obj.q)))
        error("Coordinates must be functions of time SYM('t')");
    end
end
end
end