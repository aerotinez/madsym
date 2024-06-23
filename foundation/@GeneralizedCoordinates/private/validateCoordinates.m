function validateCoordinates(obj)
    arguments
        obj (1,1) GeneralizedCoordinates;
    end
    if ~all(isDynamicVariable(obj.All))
        error('Coordinates must be dynamic variables')
    end
end