function x_ind = independent(obj)
    arguments
        obj (:,1) DynamicVariable;
    end
    x_ind = obj(~[obj.IsDependent]);
end