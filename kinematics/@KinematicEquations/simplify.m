function eomk = simplify(obj)
    arguments
        obj (1,1) KinematicEquations;
    end
    t = sym('t');
    M = obj.MassMatrix;
    f = simplify(expand(obj.ForcingVector));
    eq = M*diff(obj.States.All,t) - f;
    eomk = KinematicEquations(obj.States,eq,obj.Inputs);
    eomk.Jacobian = simplify(expand(eomk.Jacobian));
    eomk.JacobianRate = simplify(expand(obj.JacobianRate));
end