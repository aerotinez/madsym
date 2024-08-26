function eomk = simplify(obj)
    arguments
        obj (1,1) KinematicEquations;
    end
    M = obj.MassMatrix;
    f = simplify(expand(obj.ForcingVector));
    eq = M*obj.Rates - f;
    eomk = KinematicEquations(obj.States,eq,obj.Inputs);
    eomk.Jacobian = simplify(expand(eomk.Jacobian));
    eomk.JacobianRate = simplify(expand(obj.JacobianRate));
end