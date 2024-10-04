function Vbar = partial(obj,eomk,pose)
    arguments
        obj (1,1) Twist;
        eomk (1,1) KinematicEquations;
        pose (1,1) Pose = obj.Pose;
    end
    Vbar = simplify(expand(jacobian(obj.vector(pose),eomk.Inputs.state)));
end