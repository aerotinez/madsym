function eom_lin = linearize(obj)
    arguments
        obj (1,1) MechanicsEquations;
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
    
    nqi = numel(qi);
    nui = numel(ui);
    
    x = [
        q;
        u;
        v;
        ];

    F = obj.BodyDynamics(1).Inputs;

    z = [
        x;
        F
        ];

    eomk = simplify(linearize(obj.Kinematics,x,F));
    feomd = @(eom)simplify(linearize(eom,x,F));
    eomd = arrayfun(feomd,obj.BodyDynamics);
    eomd = sum(eomd);
    
    eqnsk = sym(eomk);
    eqnsd = sym(eomd);

    if ~isempty(dependent(u))
        A = obj.Constraints.Jacobian;
        Ad = obj.Constraints.JacobianRate;
        eoma = A*u.rate() + Ad*u.state(); 
        f0 = simplify(expand(subsTrim(jacobian(eoma,x.rate),z)*x.rate));
        f1 = simplify(expand(subsTrim(jacobian(eoma,x.state),z)*x.state)); 

        eqnsd = [
            eqnsd;
            f0 + f1 
            ];
    end

    if ~isempty(v)
        eomv = linearize(obj.Auxiliary,x,F);
        eqnsd = [
            eqnsd;
            sym(eomv)
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

    Pui = permMatInd(u).';
    Pud = permMatDep(u).';

    Rcq = eye(nq);
    Rkq = zeros(nq);
    Rkqd = eye(nq);
    if nq > nqi
        fc = obj.Constraints.configuration;
        Jfcq = simplify(expand(subsTrim(jacobian(fc,q.state),z)));
        Dq = -Pqd*syminv(Jfcq*Pqd);
        Rcq = simplify(expand((eye(nq) + Dq*Jfcq)*Pqi));

        fcd = diff(obj.Constraints.configuration,sym('t'));
        Jfcdq = simplify(expand(subsTrim(jacobian(fcd,q.state),z)));
        Jfcdqd = simplify(expand(subsTrim(jacobian(fcd,q.rate),z)));
        Dqd = -Pqd*syminv(Jfcdqd*Pqd);
        Rkq = simplify(expand(Dqd*Jfcdq*Rcq));
        Rkqd = simplify(expand((eye(nq) + Dqd*Jfcdqd)*Pqi));
    end

    Rvq = zeros(nq);
    Rvu = eye(nu);
    Raq = zeros(nq);
    Rau = zeros(nu);
    Raud = eye(nu);
    if nu > nui
        fv = obj.Constraints.velocity;
        Jfvq = simplify(expand(subsTrim(jacobian(fv,q.state),z)));
        Jfvu = simplify(expand(subsTrim(jacobian(fv,u.state),z)));
        Du = -Pud*syminv(Jfvu*Pud);
        Rvq = simplify(expand(Du*Jfvq*Rcq));
        Rvu = simplify(expand((eye(nu) + Du*Jfvu)*Pui));

        fa = obj.Constraints.acceleration;
        Jfaq = simplify(expand(subsTrim(jacobian(fa,q.state),z)));
        Jfau = simplify(expand(subsTrim(jacobian(fa,u.state),z)));
        Jfaud = simplify(expand(subsTrim(jacobian(fa,u.rate),z)));
        Dud = -Pud*syminv(Jfaud*Pud);
        Raq = simplify(expand(Dud*(Jfaq*Rcq + Jfau*Rvq)));
        Rau = simplify(expand(Dud*Jfau*Rvu));
        Raud = simplify(expand((eye(nu) + Dud*Jfaud)*Pui));
    end

    Rmd = blkdiag(Rkqd,Raud);

    Rm = [
        Rkq,zeros(nq,nui);
        Raq,Rau
        ];

    Rh = [
        Rcq,zeros(nq,nui);
        Rvq,Rvu
        ];

    xr = [
        qi;
        ui;
        v
        ];

    P = blkdiag(Pqi,Pui);

    Mr = P.'*M*Rmd;
    Hr = P.'*(H*Rh - M*Rm);
    Gr = P.'*G;
    
    eom_lin = LinearizedMotionEquations(xr,Mr,Hr,Gr,F);
end