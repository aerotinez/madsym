function setTrim(obj,x0,xd0)
    arguments
        obj (:,1) DynamicVariable;
        x0 (:,1) sym;
        xd0 (:,1) sym = [obj.TrimRate].';
    end
    n = numel(obj);
    if numel(x0) ~= n || numel(xd0) ~= n
        error("There must be as many trim conditions as dynamic variables");
    end
    for k = 1:n
        obj(k).TrimState = x0(k);
        obj(k).TrimRate = xd0(k);
    end
end