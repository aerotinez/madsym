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
    Vbar = simplify(expand(J*W));

    % twist in terms of generalized speeds
    twist = obj;
    twist.Vector = simplify(expand(Vbar*u.state));

    % partial velocity rate
    Jd = twist.jacobianRate(q);
    Wd = eomk.JacobianRate;
    Vdbar = simplify(expand(Jd*W + J*Wd));

    % twist in terms of generalized speeds
    twist.RateVector = simplify(expand(Vbar*u.rate + Vdbar*u.state));
end
