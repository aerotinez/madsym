function P = permMatDep(obj)
    arguments
        obj (:,1) DynamicVariable;
    end
    q = obj.state();
    qdep = obj.dependent().state();
    P = jacobian(qdep,q);
end