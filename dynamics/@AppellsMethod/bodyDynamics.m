function eomd = bodyDynamics(obj,body)
    arguments
        obj (1,1) AppellsMethod;
        body (1,1) Body;
    end
    body.reformulate(obj.Kinematics);
    u = obj.States.Speeds.Independent;
    Vbar = body.Twist.partial(obj.Kinematics);
    Vdbar = body.Twist.partialRate(obj.Kinematics);
    ad = body.Twist.Adjoint;
    G = blkdiag(body.Inertia,body.Mass*eye(3,'sym'));
    F = body.ActiveForces;
    M = Vbar.'*G*Vbar;
    f0 = -Vbar.'*G*Vdbar*u;
    f1 = Vbar.'*ad.'*G*Vbar*u;
    f2 = Vbar.'*F;
    eomd = DynamicEquations(GeneralizedCoordinates(u),M,f0,f1,f2,obj.Inputs);
end