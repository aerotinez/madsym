function f = symfun(obj)
    arguments
        obj (:,1) DynamicVariable;
    end
    f0 = arrayfun(@(x)symfun(x.state,sym('t')),obj,'uniform',0);
    f = [f0{:}].';
end