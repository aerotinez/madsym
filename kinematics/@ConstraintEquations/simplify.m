function simplify(obj)
    arguments
        obj (1,1) ConstraintEquations
    end
    prop_names = cellfun(@string,properties(obj));
    f = @(name)set(obj,name,simplify(expand((get(obj,name)))));
    arrayfun(f,prop_names);
end