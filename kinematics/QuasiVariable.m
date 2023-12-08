classdef QuasiVariable
properties (GetAccess = public, SetAccess = private)
    Velocity sym;
    Acceleration sym;
end
methods (Access = public)
function obj = QuasiVariable(variable)
    arguments
        variable sym {mustBeScalarOrEmpty} = sym.empty();
    end
    obj.Velocity = variable;
    obj.validateVariable();
    obj.Acceleration = diff(variable,sym('t'));
end
end
methods (Access = private)
function validateVariable(obj)
    if ~has(obj.Velocity,sym('t'))
        error("Variable must be a function of time SYM('t').");
    end
end
end
end