function x = partition(obj)
    arguments
        obj (:,1) DynamicVariable
    end
    x = [
        obj.independent;
        obj.dependent;
        ];
end