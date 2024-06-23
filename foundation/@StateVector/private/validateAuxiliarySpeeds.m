function validateAuxiliarySpeeds(obj)
    arguments
        obj (1,1) StateVector;
    end
    if isempty(obj.Auxiliary)
        return
    end
    if ~all(isDynamicVariable(obj.Auxiliary))
        error('Auxiliary variables must be dynamic variables');
    end
end