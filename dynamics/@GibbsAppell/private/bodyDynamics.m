function eomd = bodyDynamics(obj,body)
    arguments
        obj (1,1) GibbsAppell;
        body (1,1) Body;
    end
    u = obj.States.Speeds.Independent;
    fk = obj.Kinematics.ForcingVector;
    W = jacobian(fk,u);

    G = blkdiag(body.Inertia,body.Mass*eye(3,'sym'));
    J = body.Twist.jacobian(diff(obj.States.Coordinates.All));

    Vbar = J*W;
    Vdbar = jacobianRate(Vbar,obj.States.Coordinates.All,fk);

    V = Vbar*u;
    wm = vec2skew(V(1:3));
    vm = vec2skew(V(4:6));
    ad = [
        wm,zeros(3);
        vm,wm
        ];

    F = body.ActiveForces;

    M = Vbar.'*G*Vbar;
    f = Vbar.'*F - Vbar.'*(G*Vdbar - ad.'*G*Vbar)*u;
    eomd = MotionEquations(u,M,f,obj.Inputs); 
end