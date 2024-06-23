function validateDependentCoordinates(obj)
    arguments
        obj (1,1) GeneralizedCoordinates;
    end
    if isempty(obj.Dependent)
        return
    end
    if ~all(ismember(obj.Dependent,obj.All)) 
        error('Dependent coordinates must be member of all coordinates')
    end
end