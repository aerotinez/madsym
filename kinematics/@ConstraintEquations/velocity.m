function eqns = velocity(obj)
    arguments
        obj (1,1) ConstraintEquations
    end
    eqns = obj.Jacobian*speeds(obj);
end