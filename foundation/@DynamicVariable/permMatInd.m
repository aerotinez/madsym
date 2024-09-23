function P = permMatInd(obj)
    arguments
        obj (:,1) DynamicVariable;
    end
    q = obj.state();
    qind = obj.independent().state();
    P = jacobian(qind,q);
end