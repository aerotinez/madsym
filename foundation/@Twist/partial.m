function Vbar = partial(obj,eomk)
    arguments
        obj (1,1) Twist;
        eomk (1,1) MotionEquations;
    end
    u = eomk.Inputs;
    Vbar = jacobian(obj.Vector,u);
end