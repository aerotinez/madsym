function eqns = acceleration(obj)
    arguments
        obj (1,1) ConstraintEquations
    end
    t = sym('t');
    eqns = obj.Jacobian*diff(speeds(obj),t) + obj.JacobianRate*speeds(obj);
end