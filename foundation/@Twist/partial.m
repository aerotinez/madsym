function Vbar = partial(obj,eomk)
    arguments
        obj (1,1) Twist;
        eomk (1,1) KinematicEquations;
    end
    u = eomk.Inputs;
    Vbar = simplify(expand(jacobian(obj.Vector,u)));
end