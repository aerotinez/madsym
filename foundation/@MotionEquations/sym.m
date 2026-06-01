function eqns = sym(obj,options)
    arguments
        obj (1,1) MotionEquations;
        options.SmallAng (:,1) = sym.empty(0,1);
    end

    f = @(x)x;
    if ~isempty(options.SmallAng)
        f = @(x)smallang(x,options.SmallAng);
    end

    if obj.IsTrimmed
        eqns = f(obj.MassMatrix*[obj.States.TrimRate].') - obj.ForcingVector;
        return
    end
    eqns = f(obj.MassMatrix*obj.States.rate()) - obj.ForcingVector;
end