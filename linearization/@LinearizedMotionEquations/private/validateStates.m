function validateStates(obj)
    arguments
        obj (1,1) LinearizedMotionEquations
    end
    if ~all(isDynamicVariable(obj.States))
        error('States must be dynamic variables.')
    end
end