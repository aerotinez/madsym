function twist = reformulate(obj,eomk)
    arguments
        obj (1,1) Twist;
        eomk (1,1) KinematicEquations;
    end
    % generalized coordinates
    q = eomk.States;

    % generalized speeds
    u = eomk.Inputs;

    % partial velocity
    J = obj.jacobian(q);
    W = eomk.Jacobian;
    Vbar = J*W;

    % twist in terms of generalized speeds
    twist = obj;
    twist.Vector = Vbar*u.state;

    % partial velocity rate
    Jd = twist.jacobianRate(q);
    Wd = eomk.JacobianRate;
    Vdbar = Jd*W + J*Wd;

    % twist in terms of generalized speeds
    twist.RateVector = Vbar*u.rate + Vdbar*u.state;
end
