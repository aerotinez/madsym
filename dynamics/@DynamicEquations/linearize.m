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

    u0 = [obj.States.TrimState].';
    du0 = [obj.States.TrimRate].';

    J0 = subsTrim(J,vars);
    dJ0 = subsTrim(dJ,vars);
    adw0 = subsTrim(adw,vars);
    W0 = subsTrim(W,vars);

    DqJ0 = subsTrim(matdiff(J,q),vars);
    DqJ0t = permute(DqJ0,[2,1,3]);
    DqdJ0 = subsTrim(matdiff(dJ,q),vars);
    Dqadw0 = subsTrim(matdiff(adw,q),vars);
    DqW0 = subsTrim(jacobian(W,q),vars);

    DudJ0 = subsTrim(matdiff(dJ,u),vars);
    Duadw0 = subsTrim(matdiff(adw,u),vars);
    DuW0 = subsTrim(jacobian(W,u),vars);

    if numel(dependent(obj.States)) < 1
        df0dq = tprod(DqJ0t,G*J0*du0) + J0.'*G*tprod(DqJ0,du0);
        df1dq = tprod(DqJ0t,G*dJ0*du0) + J0.'*G*trpod(DqdJ0,u0);
        df2dq = tprod(DqJ0t,adw0*G*J0*u0) + J0.'*trpod(Dqadw0,G*J0*u0) + J0.'*adw0*G*tprod(DqJ0,u0);
        df3dq = -tprod(DqJ0t,W0) - J0.'*DqW0;

        df1du = J0.'*G*tprod(DudJ0,u0) + J0.'*G*dJ0;
        df2du = J0.'*tprod(Duadw0,G*J0*u0) + J0.'*adw0*G*J0;
        df3du = -J0.'*DuW0;

        Mlin = [zeros(nu,nq),J0.'*G*J0];
        Hlin = -[df0dq + df1dq + df2dq + df3dq,df1du + df2du + df3du];
        Glin = -subsTrim(jacobian(-J.'*W,state(F)),vars);
    else
        ui = state(independent(obj.States));
        ud = state(dependent(obj.States));
    
        % Your jacobian(state(u),state(ui)) gives nu x nui,
        % but the equations use row selectors, so transpose them.
        Si = jacobian(u,ui).';
        Sd = jacobian(u,ud).';
    
        Jc = constrainingJacobian(obj);
    
        Si0 = subsTrim(Si,vars);
        Sd0 = subsTrim(Sd,vars);
        Jc0 = subsTrim(Jc,vars);
    
        DqJc0t = subsTrim(matdiff(Jc.',q),vars);
    
        % Useful repeated blocks
        B0 = J0.'*G*J0;
        B1 = J0.'*G*dJ0;
        B2 = J0.'*adw0*G*J0;
    
        % df0 / dq
        df0dq_i = Si0*tprod(DqJ0t,G*J0*du0) ...
                + Si0*J0.'*G*tprod(DqJ0,du0);
    
        df0dq_d = tprod(DqJc0t,Sd0*B0*du0) ...
                + Jc0.'*Sd0*tprod(DqJ0t,G*J0*du0) ...
                + Jc0.'*Sd0*J0.'*G*tprod(DqJ0,du0);
    
        df0dq = df0dq_i + df0dq_d;
    
        % df1 / dq
        df1dq_i = Si0*tprod(DqJ0t,G*dJ0*u0) ...
                + Si0*J0.'*G*tprod(DqdJ0,u0);
    
        df1dq_d = tprod(DqJc0t,Sd0*B1*u0) ...
                + Jc0.'*Sd0*tprod(DqJ0t,G*dJ0*u0) ...
                + Jc0.'*Sd0*J0.'*G*tprod(DqdJ0,u0);
    
        df1dq = df1dq_i + df1dq_d;
    
        % df2 / dq
        df2dq_i = Si0*tprod(DqJ0t,adw0*G*J0*u0) ...
                + Si0*J0.'*tprod(Dqadw0,G*J0*u0) ...
                + Si0*J0.'*adw0*G*tprod(DqJ0,u0);
    
        df2dq_d = tprod(DqJc0t,Sd0*B2*u0) ...
                + Jc0.'*Sd0*tprod(DqJ0t,adw0*G*J0*u0) ...
                + Jc0.'*Sd0*J0.'*tprod(Dqadw0,G*J0*u0) ...
                + Jc0.'*Sd0*J0.'*adw0*G*tprod(DqJ0,u0);
    
        df2dq = df2dq_i + df2dq_d;
    
        % df3 / dq
        df3dq_i = -Si0*tprod(DqJ0t,W0) ...
                  -Si0*J0.'*DqW0;
    
        df3dq_d = -tprod(DqJc0t,Sd0*J0.'*W0) ...
                  -Jc0.'*Sd0*tprod(DqJ0t,W0) ...
                  -Jc0.'*Sd0*J0.'*DqW0;
    
        df3dq = df3dq_i + df3dq_d;
    
        % df / du
        df1du = Si0*J0.'*G*tprod(DudJ0,u0) ...
              + Si0*J0.'*G*dJ0 ...
              + Jc0.'*Sd0*J0.'*G*tprod(DudJ0,u0) ...
              + Jc0.'*Sd0*J0.'*G*dJ0;
    
        df2du = Si0*J0.'*tprod(Duadw0,G*J0*u0) ...
              + Si0*J0.'*adw0*G*J0 ...
              + Jc0.'*Sd0*J0.'*tprod(Duadw0,G*J0*u0) ...
              + Jc0.'*Sd0*J0.'*adw0*G*J0;
    
        df3du = -Si0*J0.'*DuW0 ...
                -Jc0.'*Sd0*J0.'*DuW0;
    
        % Mass-like linear term wrt du
        Mdu = Si0*J0.'*G*J0 ...
            + Jc0.'*Sd0*J0.'*G*J0;
    
        Mlin = [zeros(size(Mdu,1),nq),Mdu];
    
        Hlin = -[df0dq + df1dq + df2dq + df3dq,df1du + df2du + df3du];
    
        Csym = Si + Jc.'*Sd;
    
        Glin = -subsTrim(jacobian(-Csym*J.'*W,state(F)),vars);
    end

    eoml = LinearizedMotionEquations(x,Mlin,Hlin,Glin,F);
end

function DA = matdiff(A,x)
    DA = zeros([size(A),numel(x)],'sym');
    for k = 1:size(DA,1)
        DA(k,:,:) = reshape(jacobian(A(k,:),x),1,size(DA,2),[]);
    end
end

function J = tprod(T,x)
    J = sym(zeros(size(T,1),size(T,3)));
    for k = 1:size(T,3)
        J(:,k) = T(:,:,k)*x;
    end
end