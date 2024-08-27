function eomd = dynamics(obj,eomk,inputs)
    arguments
        obj (1,1) Body;
        eomk (1,1) KinematicEquations;
        inputs (:,1) sym = sym.empty(0,1);
    end
    % generalized coordinates and speeds
    qd = eomk.Rates;
    u = eomk.Inputs;

    % reforumate twist 
    V = obj.Twist.reformulate(eomk);
    ad = V.adjoint();

    % partial velocity and rate
    Vbar = V.partial(eomk);
    Vdbar = V.partialRate(eomk);

    % mass matrix
    G = blkdiag(obj.Inertia,obj.Mass.*eye(3));
    M = Vbar.'*G*Vbar;

    % inertial forces
    f0 = -Vbar.'*G*Vdbar*u;
    f1 = Vbar.'*ad.'*G*Vbar*u;

    % active forces
    f2 = Vbar.'*subs(obj.ActiveForces,qd,eomk.ForcingVector);

    % equations of motion
    eomd = DynamicEquations(u,M,f0,f1,f2,inputs);
end