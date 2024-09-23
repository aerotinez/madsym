function disp(obj)
    arguments
        obj (:,1) DynamicVariable;
    end
    disp([obj.State]);
end