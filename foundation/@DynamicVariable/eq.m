function res = eq(xa,xb)
    arguments
        xa (:,1) DynamicVariable;
        xb (:,1) DynamicVariable;
    end
    if numel(xa) == numel(xb)
        res = arrayfun(@isequal,xa,xb);
        return
    end
    if isscalar(xa)
        res = arrayfun(@(x)isequal(x,xa),xb);
        return
    end
    if isscalar(xb)
        res = arrayfun(@(x)isequal(x,xb),xa);
        return
    end
    msga = "Arguments must either have equal number of elements or ";
    msgb = "one argument must have one element only.";
    error(msga + msgb);
end