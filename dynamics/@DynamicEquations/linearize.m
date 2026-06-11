function eoml = linearize(obj,x,F,options)
    arguments
        obj (1,1) DynamicEquations;
        x (:,1) DynamicVariable = obj.States;
        F (:,1) DynamicVariable = obj.Inputs;
        options.SmallAngs (:,1) DynamicVariable = DynamicVariable.empty(0,1);
    end

    strat = @(f)f;
    if ~isempty(options.SmallAngs)
        strat = @(f)smallang(f,state(options.SmallAngs));
    end

    strat0 = @(f)f;
    if ~isempty(options.SmallAngs)
        strat0 = @(f)smallang(f,[options.SmallAngs.TrimState].');
    end

    vars = [
        x;
        F
        ];

    nu = numel(obj.States);
    nq = numel(x) - nu;

    q = state(x(1:nq));
    u = state(obj.States);

    G  = obj.SpatialInertia;
    J  = strat(obj.Jacobian);
    dJ = strat(obj.JacobianRate);
    W  = strat(obj.ActiveForces);

    V = strat(J*u);

    nb = size(G,1)/6;
    adw = sym.zeros(6*nb,6*nb);

    for i = 1:nb
        idx = (6*(i-1)+1):(6*i);
        w = V(idx(1:3));
        adw(idx,idx) = blkdiag(vec2skew(w),vec2skew(w));
    end

    u0  = [obj.States.TrimState].';
    du0 = [obj.States.TrimRate].';

    J0    = subsTrim(J,vars);
    dJ0   = subsTrim(dJ,vars);
    adw0  = subsTrim(adw,vars);
    W0    = subsTrim(W,vars);

    DqJ0   = strat0(subsTrim(matdiff(J,q),vars));
    DqJ0t  = permute(DqJ0,[2,1,3]);
    DqdJ0  = strat0(subsTrim(matdiff(dJ,q),vars));
    Dqadw0 = strat0(subsTrim(matdiff(adw,q),vars));
    DqW0   = strat0(subsTrim(jacobian(W,q),vars));

    DudJ0  = strat0(subsTrim(matdiff(dJ,u),vars));
    Duadw0 = strat0(subsTrim(matdiff(adw,u),vars));
    DuW0   = strat0(subsTrim(jacobian(W,u),vars));

    Jt0 = J0.';
    JG0 = Jt0*G;

    if numel(dependent(obj.States)) < 1
        GJ0du0    = G*J0*du0;
        GdJ0u0    = G*dJ0*u0;
        adwGJ0u0  = adw0*G*J0*u0;
        GJ0u0     = G*J0*u0;

        JGJ0 = JG0*J0;

        df0dq = tprod(DqJ0t,GJ0du0) ...
              + JG0*tprod(DqJ0,du0);

        df1dq = tprod(DqJ0t,GdJ0u0) ...
              + JG0*tprod(DqdJ0,u0);

        df2dq = tprod(DqJ0t,adwGJ0u0) ...
              + Jt0*tprod(Dqadw0,GJ0u0) ...
              + Jt0*adw0*G*tprod(DqJ0,u0);

        df3dq = -tprod(DqJ0t,W0) ...
                -Jt0*DqW0;

        df1du = JG0*tprod(DudJ0,u0) ...
              + JG0*dJ0;

        df2du = Jt0*tprod(Duadw0,GJ0u0) ...
              + Jt0*adw0*G*J0;

        df3du = -Jt0*DuW0;

        Mlin = [zeros(nu,nq),JGJ0];

        Hlin = -[df0dq + df1dq + df2dq + df3dq, ...
                 df1du + df2du + df3du];

        Glin = -subsTrim(jacobian(-Jt0*W,state(F)),vars);

    else
        ui = state(independent(obj.States));
        ud = state(dependent(obj.States));

        Si = jacobian(u,ui).';
        Sd = jacobian(u,ud).';

        Jc = strat(obj.ConstraintJacobian);

        Si0 = subsTrim(Si,vars);
        Sd0 = subsTrim(Sd,vars);
        Jc0 = strat0(subsTrim(Jc,vars));

        DqJc0t = strat0(subsTrim(matdiff(Jc.',q),vars));

        P0  = strat0(Si0 + Jc0.'*Sd0);
        PJ0 = strat0(P0*Jt0);
        PJG0 = strat0(PJ0*G);

        B0 = strat0(JG0*J0);
        B1 = strat0(JG0*dJ0);
        B2 = strat0(Jt0*adw0*G*J0);

        PJGJ0   = strat0(PJG0*J0);
        PJadGJ0 = strat0(PJ0*adw0*G*J0);

        GJ0du0   = strat0(G*J0*du0);
        GdJ0u0   = strat0(G*dJ0*u0);
        adwGJ0u0 = strat0(adw0*G*J0*u0);
        GJ0u0    = strat0(G*J0*u0);

        c0 = strat0(Sd0*B0*du0);
        c1 = strat0(Sd0*B1*u0);
        c2 = strat0(Sd0*B2*u0);
        c3 = strat0(Sd0*Jt0*W0);

        df0dq = strat0(P0*tprod(DqJ0t,GJ0du0)) ...
              + strat0(PJG0*tprod(DqJ0,du0)) ...
              + strat0(tprod(DqJc0t,c0));

        df1dq = strat0(P0*tprod(DqJ0t,GdJ0u0)) ...
              + strat0(PJG0*tprod(DqdJ0,u0)) ...
              + strat0(tprod(DqJc0t,c1));

        df2dq = strat0(P0*tprod(DqJ0t,adwGJ0u0)) ...
              + strat0(PJ0*tprod(Dqadw0,GJ0u0)) ...
              + strat0(PJadGJ0*tprod(DqJ0,u0)) ...
              + strat0(tprod(DqJc0t,c2));

        df3dq = -strat0(P0*tprod(DqJ0t,W0)) ...
                -strat0(PJ0*DqW0) ...
                -strat0(tprod(DqJc0t,c3));

        df1du = strat0(PJG0*tprod(DudJ0,u0)) ...
              + strat0(PJG0*dJ0);

        df2du = strat0(PJ0*tprod(Duadw0,GJ0u0)) ...
              + strat0(PJadGJ0);

        df3du = -strat0(PJ0*DuW0);

        Mdu = PJGJ0;

        Mlin = [zeros(size(Mdu,1),nq),Mdu];

        Hlin = -[df0dq + df1dq + df2dq + df3dq, ...
                 df1du + df2du + df3du];

        Csym = Si + Jc.'*Sd;

        Glin = -strat0(subsTrim(jacobian(-Csym*J.'*W,state(F)),vars));
    end

    eoml = LinearizedMotionEquations(x,Mlin,Hlin,Glin,F);
end