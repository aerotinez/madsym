function x_dep = dependent(obj)
    arguments
        obj (:,1) DynamicVariable;
    end
    x_dep = obj([obj.IsDependent]);
end