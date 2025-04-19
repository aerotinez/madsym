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

    eomk = linearize(obj.Kinematics,x,F);
    feomd = @(eom)linearize(eom,x,F);
    eomd = sum(arrayfun(feomd,obj.BodyDynamics));
    
    eqnsk = sym(eomk);
    eqnsd = sym(eomd);

    if ~isempty(dependent(u))
        A = obj.Constraints.Jacobian;
        Ad = obj.Constraints.JacobianRate;
        eoma = A*u.rate() + Ad*u.state(); 
        f0 = subsTrim(jacobian(eoma,x.rate),z)*x.rate;
        f1 = subsTrim(jacobian(eoma,x.state),z)*x.state; 

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

    if nq == nqi && nu == nui
        eom_lin = LinearizedMotionEquations(x,M,H,G,F);
        return
    end

    Pqi = permMatInd(q).';
    Pqd = permMatDep(q).';

    Pui = permMatInd(u).';
    Pud = permMatDep(u).';

    Rcq = eye(nq,'sym');
    if nq > nqi
        fc = obj.Constraints.configuration;
        Jfcq = subsTrim(jacobian(fc,q.state),z);
        Dq = simplify(expand(-Pqd*syminv(Jfcq*Pqd)));
        Rcq = simplify(expand((eye(nq) + Dq*Jfcq)*Pqi));
    end

    fv = obj.Constraints.velocity;
    Jfvq = subsTrim(jacobian(fv,q.state),z);
    Jfvu = subsTrim(jacobian(fv,u.state),z);
    Du = simplify(expand(-Pud*syminv(Jfvu*Pud)));
    Rvq = simplify(expand(Du*Jfvq*Rcq));
    Rvu = simplify(expand((eye(nu) + Du*Jfvu)*Pui));

    R = [
        Rcq,zeros(nq,nui);
        Rvq,Rvu
        ];

    xr = [
        qi;
        ui;
        v
        ];

    Mr = R.'*M*R;
    Hr = R.'*H*R;
    Gr = R.'*G;

    eom_lin = LinearizedMotionEquations(xr,Mr,Hr,Gr,F);
end