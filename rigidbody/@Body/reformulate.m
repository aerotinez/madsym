function reformulate(obj,eomk)
    arguments
        obj (1,1) Body
        eomk (1,1) KinematicEquations
    end
    obj.Twist.reformulate(eomk);
    obj.InertialForces = obj.inertialForces();
    obj.ActiveForces = subs(obj.ActiveForces,eomk.Rates,eomk.ForcingVector);
end