function eomc = simplify(obj)
    arguments
        obj (1,1) ConstraintEquations
    end
    eomc = obj;
    eomc.Velocity = simplify(eomc.Velocity);
    eomc.Acceleration = simplify(eomc.Acceleration);
    eomc.Jacobian = simplify(expand(eomc.Jacobian));
    eomc.JacobianRate = simplify(expand(eomc.JacobianRate)); 
end