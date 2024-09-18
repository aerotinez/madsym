function eomc = simplify(obj)
    arguments
        obj (1,1) ConstraintEquations
    end
    eomc = obj;
    eomc.Configuration = simplify(expand(eomc.Configuration));
    eomc.Jacobian = simplify(expand(eomc.Jacobian));
    eomc.JacobianRate = simplify(expand(eomc.JacobianRate)); 
end