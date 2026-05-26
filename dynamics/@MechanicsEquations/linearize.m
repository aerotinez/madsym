function eom_lin = linearize(obj,options)
    arguments
        obj (1,1) MechanicsEquations;
        options.SmallAngs (:,1) DynamicVariable = DynamicVariable.empty(0,1);
    end

    strat = @(f)f;
    if ~isempty(options.SmallAngs)
        strat = @(f)smallang(f,[options.SmallAngs.TrimState].');
    end

    q = obj.Kinematics.States;
    u = obj.Kinematics.Inputs;
    v = DynamicVariable.empty(0,1);

    if ~isempty(obj.Auxiliary)
        v = obj.Auxiliary.States;
    end

    qi = q.independent;
    ui = u.independent;

    nq = numel(q);
    nu = numel(u);
    nv = numel(v);
    
    nqi = numel(qi);
    nui = numel(ui);
    
    x = [
        q;
        v;
        u
        ];

    F = obj.BodyDynamics(1).Inputs;

    z = [
        x;
        F
        ];

    eomk = simplify(strat(linearize(obj.Kinematics,x,F)));

    feomd = @(eom)simplify(linearize(eom,x,F));
    if ~isempty(options.SmallAngs)
        feomd = @(eom)simplify(linearize(eom,x,F,"SmallAngs",options.SmallAngs));
    end

    eomd_b = arrayfun(feomd,obj.BodyDynamics);
    eomd = sum(eomd_b);
    
    eqnsk = sym(eomk);
    eqnsd = sym(eomd);

    if ~isempty(dependent(u))
        A = obj.Constraints.Jacobian;
        Ad = obj.Constraints.JacobianRate;
        eoma = A*u.rate() + Ad*u.state(); 
        f0 = simplify(expand(strat(subsTrim(jacobian(eoma,x.rate),z))*x.rate));
        f1 = simplify(expand(strat(subsTrim(jacobian(eoma,x.state),z))*x.state)); 

        eqnsd = [
            eqnsd;
            f0 + f1 
            ];
    end

    if ~isempty(v)
        eomv = linearize(obj.Auxiliary,x,F);
        eqnsd = [
            sym(eomv)
            eqnsd
            ];
    end

    eqns = [
        eqnsk;
        eqnsd
        ];

    [M,f] = massMatrixForm(eqns,x.state);
    [H,g] = equationsToMatrix(f,x.state);
    G = jacobian(-g,F.state);

    Pqi = permMatInd(q).';
    Pqd = permMatDep(q).';

    Pui = permMatInd([v;u]).';
    Pud = permMatDep([v;u]).';

    Rcq = eye(nq);
    Rkq = zeros(nq);
    Rkqd = eye(nq);
    if nq > nqi
        fc = obj.Constraints.configuration;
        Jfcq = simplify(expand(strat(subsTrim(jacobian(fc,q.state),z))));
        Dq = -Pqd*syminv(Jfcq*Pqd);
        Rcq = simplify(strat(expand((eye(nq) + Dq*Jfcq)*Pqi)));

        fcd = diff(obj.Constraints.configuration,sym('t'));
        Jfcdq = simplify(expand(strat(subsTrim(jacobian(fcd,q.state),z))));
        Jfcdqd = simplify(expand(strat(subsTrim(jacobian(fcd,q.rate),z))));
        Dqd = -Pqd*syminv(Jfcdqd*Pqd);
        Rkq = simplify(strat(expand(Dqd*Jfcdq*Rcq)));
        Rkqd = simplify(strat(expand((eye(nq) + Dqd*Jfcdqd)*Pqi)));
    end

    Rvq = zeros(nq);
    Rvu = eye(nu + nv);
    Raq = zeros(nq);
    Rau = zeros(nu + nv);
    Raud = eye(nu + nv);
    if nu > nui
        fv = obj.Constraints.velocity;
        Jfvq = simplify(expand(strat(subsTrim(jacobian(fv,q.state),z))));
        Jfvu = simplify(expand(strat(subsTrim(jacobian(fv,state([v;u])),z))));
        Du = -Pud*syminv(Jfvu*Pud);
        Rvq = simplify(strat(expand(Du*Jfvq*Rcq)));
        Rvu = simplify(strat(expand((eye(nu + nv) + Du*Jfvu)*Pui)));

        fa = obj.Constraints.acceleration;
        Jfaq = simplify(expand(strat(subsTrim(jacobian(fa,q.state),z))));
        Jfau = simplify(expand(strat(subsTrim(jacobian(fa,state([v;u])),z))));
        Jfaud = simplify(expand(strat(subsTrim(jacobian(fa,rate([v;u])),z))));
        Dud = -Pud*syminv(Jfaud*Pud);
        Raq = simplify(strat(expand(Dud*(Jfaq*Rcq + Jfau*Rvq))));
        Rau = simplify(strat(expand(Dud*Jfau*Rvu)));
        Raud = simplify(strat(expand((eye(nu + nv) + Dud*Jfaud)*Pui)));
    end

    Rmd = blkdiag(Rkqd,Raud);

    Rm = [
        Rkq,zeros(nq,nui + nv);
        Raq,Rau
        ];

    Rh = [
        Rcq,zeros(nq,nui + nv);
        Rvq,Rvu
        ];

    xr = [
        qi;
        v;
        ui
        ];

    P = blkdiag(Pqi,Pui);

    Mr = P.'*M*Rmd;
    Hr = P.'*(H*Rh - M*Rm);
    Gr = P.'*G;
    
    eom_lin = LinearizedMotionEquations(xr,Mr,Hr,Gr,F);
end