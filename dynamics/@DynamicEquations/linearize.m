function eoml = linearize(obj,x,F)
    arguments
        obj (1,1) DynamicEquations;
        x (:,1) DynamicVariable = obj.States;
        F (:,1) DynamicVariable = obj.Inputs;
    end

    z = [
        x;
        F
        ];

    nu = numel(obj.States);
    nq = numel(x) - nu;

    q = state(x(1:nq));
    u = state(obj.States);

    G = obj.SpatialInertia;
    J = obj.Jacobian;
    dJ = obj.JacobianRate;
    W = obj.ActiveForces;

    V = J*u;

    nb = size(G,1)/6;
    adw = sym.zeros(6*nb,6*nb);

    for i = 1:nb
        idx = (6*(i-1)+1):(6*i);
        w = V(idx(1:3));
        adw(idx,idx) = blkdiag(vec2skew(w),vec2skew(w));
    end

    u0 = [obj.States.TrimState].';
    du0 = [obj.States.TrimRate].';

    J0 = subsTrim(J,z);
    dJ0 = subsTrim(dJ,z);
    adw0 = subsTrim(adw,z);
    W0 = subsTrim(W,z);

    DqJ0 = subsTrim(matdiff(J,q),z);
    DqJ0t = permute(DqJ0,[2,1,3]);
    DqdJ0 = subsTrim(matdiff(dJ,q),z);
    Dqadw0 = subsTrim(matdiff(adw,q),z);
    DqW0 = subsTrim(jacobian(W,q),z);

    DudJ0 = subsTrim(matdiff(dJ,u),z);
    Duadw0 = subsTrim(matdiff(adw,u),z);
    DuW0 = subsTrim(jacobian(W,u),z);

    DFW0 = subsTrim(jacobian(W,state(F)),z);

    if numel(dependent(obj.States)) < 1
        df0dq = tprod(DqJ0t,G*J0*du0) + J0.'*G*tprod(DqJ0,du0);
        df1dq = tprod(DqJ0t,G*dJ0*u0) + J0.'*G*tprod(DqdJ0,u0);
        df2dq = tprod(DqJ0t,adw0*G*J0*u0) + J0.'*tprod(Dqadw0,G*J0*u0) + ...
            J0.'*adw0*G*tprod(DqJ0,u0);
        df3dq = -tprod(DqJ0t,W0) - J0.'*DqW0;

        df1du = J0.'*G*tprod(DudJ0,u0) + J0.'*G*dJ0;
        df2du = J0.'*tprod(Duadw0,G*J0*u0) + J0.'*adw0*G*J0;
        df3du = -J0.'*DuW0;

        df0ddu = J0.'*G*J0;

        df3dF = -J0.'*DFW0;
    else
        ui = state(independent(obj.States));
        ud = state(dependent(obj.States));

        Si = jacobian(u,ui).';
        Sd = jacobian(u,ud).';

        Jc = obj.ConstraintJacobian;
        Jc0 = subsTrim(Jc,z);

        DqJc0t = subsTrim(matdiff(Jc.',q),z);

        df0idq = Si*tprod(DqJ0t,G*J0*du0) + Si*J0.'*G*tprod(DqJ0,du0);

        df0ddq = tprod(DqJc0t,Sd*J0.'*G*J0*du0) + ...
            Jc0.'*Sd*tprod(DqJ0t,G*J0*du0) + Jc0.'*Sd*J0.'*G*tprod(DqJ0,du0);

        df0dq = df0idq + df0ddq;
        
        df1idq = Si*tprod(DqJ0t,G*dJ0*u0) + Si*J0.'*G*tprod(DqdJ0,u0);
        
        df1ddq = tprod(DqJc0t,Sd*J0.'*G*dJ0*u0) + ...
            Jc0.'*Sd*tprod(DqJ0t,G*dJ0*u0) + Jc0.'*Sd*J0.'*G*tprod(DqdJ0,u0);

        df1dq = df1idq + df1ddq;
        
        df2idq = Si*tprod(DqJ0t,adw0*G*J0*u0) + ...
            Si*J0.'*tprod(Dqadw0,G*J0*u0) + Si*J0.'*adw0*G*tprod(DqJ0,u0);
        
        df2ddq = tprod(DqJc0t,Sd*J0.'*adw0*G*J0*u0) + ...
            Jc0.'*Sd*tprod(DqJ0t,adw0*G*J0*u0) + ...
            Jc0.'*Sd*J0.'*tprod(Dqadw0,G*J0*u0) + ...
            Jc0.'*Sd*J0.'*adw0*G*tprod(DqJ0,u0);

        df2dq = df2idq + df2ddq;
        
        df3idq = -Si*tprod(DqJ0t,W0) - Si*J0.'*DqW0;
        
        df3ddq = -tprod(DqJc0t,Sd*J0.'*W0) - Jc0.'*Sd*tprod(DqJ0t,W0) - ...
            Jc0.'*Sd*J0.'*DqW0;

        df3dq = df3idq + df3ddq;
        
        df1idu = Si*J0.'*G*tprod(DudJ0,u0) + Si*J0.'*G*dJ0;
        df1ddu = Jc0.'*Sd*J0.'*G*tprod(DudJ0,u0) + Jc0.'*Sd*J0.'*G*dJ0;
        df1du = df1idu + df1ddu;

        df2idu = Si*J0.'*tprod(Duadw0,G*J0*u0) + Si*J0.'*adw0*G*J0;
        df2ddu = Jc0.'*Sd*J0.'*tprod(Duadw0,G*J0*u0) + ...
            Jc0.'*Sd*J0.'*adw0*G*J0;
        df2du = df2idu + df2ddu;
        
        df3idu = -Si*J0.'*DuW0;
        df3ddu = -Jc0.'*Sd*J0.'*DuW0;
        df3du = df3idu + df3ddu;

        df0iddu = Si*J0.'*G*J0;
        df0dddu = Jc0.'*Sd*J0.'*G*J0;
        df0ddu = df0iddu + df0dddu;

        df3idF = -Si*J0.'*DFW0;
        df3ddF = -Jc0.'*Sd*J0.'*DFW0;
        df3dF = df3idF + df3ddF;
    end

    Mlin = [zeros(size(df0ddu,1),nq),df0ddu];
    Hlin = -[df0dq + df1dq + df2dq + df3dq,df1du + df2du + df3du];
    Glin = -df3dF;

    eoml = LinearizedMotionEquations(x,Mlin,Hlin,Glin,F);
end