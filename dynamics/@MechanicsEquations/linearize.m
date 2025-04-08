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

    nq = numel(q);
    nu = numel(u);
    nv = numel(v);
    
    x = [
        q;
        u;
        v;
        ];

    F = obj.BodyDynamics(1).Inputs;

    eomk = linearize(obj.Kinematics,x,F);
    feomd = @(eom)linearize(eom,x,F);
    eomd = sum(arrayfun(feomd,obj.BodyDynamics));
    
    eqns = [
        sym(eomk);
        sym(eomd)
        ];

    if ~isempty(dependent(u))
        A = obj.Constraints.Jacobian;
        Ad = obj.Constraints.JacobianRate;
        eoma = A*u.rate() + Ad*u.state(); 
        f0 = subsTrim(jacobian(eoma,x.rate),[x;F])*x.rate;
        f1 = subsTrim(jacobian(eoma,x.state),[x;F])*x.state; 

        eqns = [
            eqns;
            f0 + f1 
            ];
    end

    if ~isempty(v)
        eomv = linearize(obj.Auxiliary,x,F);
        eqns = [
            eqns;
            sym(eomv)
            ];
    end

    [M,f] = massMatrixForm(eqns,x.state);
    [H,g] = equationsToMatrix(f,x.state);
    G = jacobian(-g,F.state);
 
    n = numel(q);
    if numel(dependent(u)) > 0 
        [C1,C2] = dependentVelocityProjection(q,[u;v],obj.Constraints);
        C1 = subsTrim(C1,[x;F]);
        C2 = subsTrim(C2,[x;F]);
        Hq = H(:,1:n);
        Hu = H(:,n + 1:end);
        H = [Hq + Hu*C1,Hu*C2];
    end

    if numel(dependent(q)) > 0
        C0 = dependentCoordinateProjection(q,obj.Constraints);
        C0 = subsTrim(C0,[x;F]);
        Hq = H(:,1:n);
        Hu = H(:,n + 1:end);
        H = [Hq*C0,Hu];
    end

    eqns = M*x.rate - H*x.independent.state - G*F.state;

    if numel(dependent(u)) > 0
        eqns_q = eqns(1:nq);
        eqns_u = eqns(nq + 1:nq + nu);

        eqns_udep = eqns_u(end - numel(u.dependent) + 1:end);
        [Mu,fu] = massMatrixForm(eqns_udep,u.dependent.state);
        ud_dep = simplify(expand(syminv(Mu)*fu));

        udi = u.independent.rate;
        Ju = jacobian([udi;ud_dep],udi);

        eqns_ui = Ju.'*subs(eqns_u,u.dependent.rate,ud_dep);

        eqns_ind = [
            eqns_q;
            eqns_ui;
            ];

        if nv > 0
            eqns_ind = [
                eqns_ind;
                subs(eqns(end - nv:end),u.dependent.rate,ud_dep)
                ];
        end

        x = [
            q;
            u.independent;
            v
            ];

        eqns = eqns_ind;
    end

    if numel(dependent(q)) > 0
        eqns_q = eqns(1:nq);
        Jq = jacobian(q.state,q.independent.state);

        x = [
            q.independent;
            x(nq + 1:end)
            ];

        eqns = [
            Jq.'*eqns_q;
            eqns(nq + 1:end);
            ];
    end

    [M,f] = massMatrixForm(eqns,x.state);
    [H,g] = equationsToMatrix(f,x.state);
    G = jacobian(-g,F.state);

    eom_lin = LinearizedMotionEquations(x,M,H,G,F);
end