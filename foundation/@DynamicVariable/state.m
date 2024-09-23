function x = state(obj)
    arguments
        obj (:,1) DynamicVariable;
    end
    x = [obj.State].';
end