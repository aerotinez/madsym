function reformulate(obj,eomk)
    arguments
        obj (1,1) Twist;
        eomk (1,1) MotionEquations;
    end
    qd = eomk.Rates;
    u = eomk.Inputs;
    fk = eomk.ForcingVector;
    prop_names = cellfun(@string,properties(obj));
    f = @(name)set(obj,name,simplify(expand(subs(get(obj,name),qd,fk))));
    arrayfun(f,prop_names(1:end - 1));
    Vbar = obj.partial(eomk);
    Vdbar = obj.partialRate(eomk);
    obj.Rate = Vbar*diff(u) + Vdbar*u;
end