function constraints(obj,eomc)
    arguments
        obj (1,1) DynamicsMethod;
        eomc (1,1) ConstraintEquations;
    end
    obj.Constraints = eomc;
end