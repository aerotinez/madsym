function Vbar = partial(obj,eomk,pose)
    arguments
        obj (1,1) Twist;
        eomk (1,1) KinematicEquations;
        pose (1,1) Pose = obj.Pose;
    end
    V = obj.vector(pose);
    Vbar = jacobian(V,eomk.Inputs.All);
end