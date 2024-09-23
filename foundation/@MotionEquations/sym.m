function eqns = sym(obj)
    arguments
        obj (1,1) MotionEquations;
    end
    eqns = obj.MassMatrix*obj.States.rate() - obj.ForcingVector;
end