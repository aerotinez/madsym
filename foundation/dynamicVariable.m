function x = dynamicVariable(name)
    arguments
        name {mustBeText};
    end
    t = sym('t');
    f = symfun(sprintf("%s(t)",name),t);
    x = f(t);
end