function eom = motionEquations(obj,eomk)
    arguments
        obj (1,1) Body;
        eomk (1,1) MotionEquations;
    end
    qd = eomk.Rates;
    u = eomk.Inputs;
    fk = eomk.ForcingVector;
    reformulate(obj.Twist,eomk);
    Vbar = partial(obj.Twist,eomk);
    Vdbar = partialRate(obj.Twist,eomk);
    ad = obj.Twist.Adjoint;
    G = blkdiag(obj.Inertia,obj.Mass*eye(3));
    F = subs(obj.ActiveForces,qd,fk);
    M = Vbar.'*G*Vbar;
    f0 = -Vbar.'*G*Vdbar*u;
    f1 = Vbar.'*ad.'*G*Vbar*u;
    f2 = Vbar.'*F;
    f = f0 + f1 + f2;
    eom = MotionEquations(u,M,f);
end