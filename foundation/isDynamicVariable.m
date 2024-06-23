function result = isDynamicVariable(in)
    arguments
        in;
    end
    result = arrayfun(@checkIsDynamicVariable,in);
end

function result = checkIsDynamicVariable(in)
    vars = findSymType(in,"symfunOf",sym('t'));
    if numel(vars) ~= 1
        result = false;
        return;
    end
    if ~isequal(in,vars)
        result = false;
        return;
    end
    result = true;
end