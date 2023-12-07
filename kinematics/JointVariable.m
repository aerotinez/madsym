classdef JointVariable < handle
properties (GetAccess = public, SetAccess = private)
    Position sym;
    Velocity sym;
    Acceleration sym;
    Stiffness (1,1) sym = sym(0);
end
properties (Access = private)
    IS_STIFFNESS_SET (1,1) logical = false;
end
methods (Access = public)
function obj = JointVariable(variable,options)
    arguments
        variable (1,1) sym;
        options.Stiffness (1,1) sym = sym(0);
    end
    obj.Position = variable;
    obj.validateVariable();
    obj.Velocity = diff(variable,sym('t'));
    obj.Acceleration = diff(obj.Velocity,sym('t'));
    if options.Stiffness ~= sym(0)
        obj.IS_STIFFNESS_SET = true;
    end
    obj.Stiffness = options.Stiffness;
end
function setStiffness(obj,stiffness)
    arguments
        obj (1,1) JointVariable;
        stiffness (1,1) sym;
    end
    if obj.IS_STIFFNESS_SET
        warning("Stiffness is already set.");
        return
    end
    obj.Stiffness = stiffness;
    obj.IS_STIFFNESS_SET = true;
end
end
methods (Access = private)
function validateVariable(obj)
    if ~has(obj.Position,sym('t'))
        error("Variable must be a function of time SYM('t').");
    end
end
end
end