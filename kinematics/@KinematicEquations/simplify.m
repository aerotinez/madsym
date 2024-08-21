function simplify(obj)
    arguments
        obj (1,1) KinematicEquations;
    end
    simplify@MotionEquations(obj);
    obj.Jacobian = simplify(expand(obj.Jacobian));
end