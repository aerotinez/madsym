function validateInputs(obj)
    arguments
        obj (1,1) MotionEquations;
    end
    if isempty(obj.Inputs)
        return
    end
    if ~all(isDynamicVariable(obj.Inputs))
        error('Inputs must be dynamic variables.')
    end
end