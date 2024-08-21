function toODE(obj)
    arguments
        obj (1,1) KinematicEquations;
    end
    toODE@MotionEquations(obj); 
    obj.Jacobian = jacobian(obj.ForcingVector,obj.Inputs);
end