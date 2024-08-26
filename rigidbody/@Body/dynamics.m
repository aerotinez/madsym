function eomd = dynamics(obj,eomk)
    arguments
        obj (1,1) Body;
        eomk (1,1) KinematicEquations;
    end

    % generalized coordinates and speeds
    q = eomk.States;
    u = eomk.Inputs;

    % partial velocity
    J = obj.Twist.jacobian(q);
    W = eomk.Jacobian;
    Vbar = J*W;

    % partial velocity rate
    Jd = obj.Twist.jacobianRate(q,eomk.ForcingVector);
    Wd = eomk.JacobianRate;
    Vdbar = Jd*W + J*Wd;

    % spatial inertia matrix
    R = obj.ReferenceFrame.dcm();
    I = simplify(expand(R.'*obj.Inertia*R));
    G = blkdiag(I,obj.Mass.*eye(3));

    % velocity adjoint matrix
    V = Vbar*u;
    wm = vec2skew(V(1:3));
    vm = vec2skew(V(4:6));
    ad = [
        wm,zeros(3);
        vm,wm
        ];

    % mass matrix
    M = Vbar.'*G*Vbar;

    % forcing vector
    fi = -Vbar.'*(G*Vdbar - ad.'*G*Vbar)*u;
    fa = Vbar.'*obj.ActiveForces.vector();
    f = fi + fa;

    % equations of motion
    eomd = MotionEquations(u,M,f);
end