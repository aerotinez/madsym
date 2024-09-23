function x = rate(obj)
    arguments
        obj (:,1) DynamicVariable;
    end
    x = [obj.Rate].';
end