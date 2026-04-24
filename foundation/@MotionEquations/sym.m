function eqns = sym(obj)
    arguments
        obj (1,1) MotionEquations;
    end
    if obj.IsTrimmed
        eqns = obj.MassMatrix*[obj.States.TrimRate].' - obj.ForcingVector;
        return
    end
    eqns = obj.MassMatrix*obj.States.rate() - obj.ForcingVector;
end