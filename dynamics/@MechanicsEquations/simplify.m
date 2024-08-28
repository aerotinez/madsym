function eom = simplify(obj)
    arguments
        obj (1,1) MechanicsEquations;
    end
    x = obj.StateVector;
    eomk = simplify(obj.Kinematics);
    eomd_list = arrayfun(@simplify,obj.BodyDynamics);
    eomc = ConstraintEquations.empty(0,1);
    if ~isempty(obj.Constraints)
        eomc = simplify(obj.Constraints);
    end
    eom = MechanicsEquations(x,eomk,eomd_list,eomc);
end