function reformulate(obj,eomk)
    arguments
        obj (1,1) ConstraintEquations
        eomk (1,1) KinematicEquations
    end
    t = sym('t');
    qd = diff(eomk.States,t);
    fsub = @(eq)subs(eq,qd,eomk.ForcingVector);
    obj.Velocity = fsub(obj.Velocity);
    obj.Acceleration = fsub(diff(obj.Velocity,t));
end