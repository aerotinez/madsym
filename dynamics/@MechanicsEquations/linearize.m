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

    eom_lin = LinearizedMotionEquations(x,M,H,G,F);
end