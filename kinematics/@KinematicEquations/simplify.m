function eomk = simplify(obj)
    arguments
        obj (1,1) KinematicEquations;
    end
    kdes = obj.States.rate() - simplify(expand(obj.ForcingVector));
    eomk = KinematicEquations(obj.States,kdes,obj.Inputs);
    eomk.Jacobian = simplify(expand(eomk.Jacobian));
    eomk.JacobianRate = simplify(expand(obj.JacobianRate));
end