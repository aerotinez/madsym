function eom = simplify(obj)
    arguments
        obj (1,1) MechanicsEquations;
    end
    eomk = simplify(obj.Kinematics);
    eomd_list = arrayfun(@simplify,obj.BodyDynamics);
    eomc = ConstraintEquations.empty(0,1);
    if ~isempty(obj.Constraints)
        eomc = simplify(obj.Constraints);
    end
    eomv = MotionEquations.empty(0,1);
    if ~isempty(obj.Auxiliary)
        eomv = simplify(obj.Auxiliary);
    end
    eom = MechanicsEquations(eomk,eomd_list,eomc,eomv);
end