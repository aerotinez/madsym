function disp(obj)
    arguments
        obj (:,1) DynamicVariable;
    end
    pprint([obj.State]);
end