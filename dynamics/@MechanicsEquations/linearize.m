function eom_lin = linearize(obj)
    arguments
        obj (1,1) MechanicsEquations;
    end

    t = sym('t');

    q = obj.StateVector.Coordinates;
    u = obj.Kinematics.Inputs;

    X = [
        q;
        u
        ];

    x = [
        diff(X.All,t);
        X.All;
        obj.Inputs.All
        ];
    
    x0 = [
        X.TrimRate;
        X.Trim;
        obj.Inputs.Trim
        ];

    eomk = linearize(obj.Kinematics);
    eomd = sum(arrayfun(@(eom)linearize(eom,obj.Kinematics),obj.BodyDynamics));
 
    Mk = eomk.MassMatrix;
    Md = eomd.MassMatrix;

    M = [
        Mk,zeros(size(Mk,1),size(Md,2) - size(Mk,2));
        eomd.MassMatrix;
        ];

    H = [
        eomk.ForcingMatrix,eomk.InputMatrix;
        eomd.ForcingMatrix;
        ];

    G = [
        zeros(size(eomk.ForcingMatrix,1),size(eomd.InputMatrix,2));
        eomd.InputMatrix;
        ];
        
    if obj.StateVector.p > 0
        eoma = linearize(obj.Auxiliary);

        M = [
            M,zeros(size(M,1),size(eoma.MassMatrix,1));
            eoma.MassMatrix
            ];

        H = [
            H,zeros(size(H,1),size(eoma.ForcingMatrix,1));
            eoma.ForcingMatrix
            ];
        
        G = [
            G;
            eoma.InputMatrix
            ];

        v = obj.StateVector.Auxiliary;

        X = [
            X;
            v
            ];

        x = [
            x;
            diff(v.All,t);
            v.All
            ];

        x0 = [
            x0;
            v.TrimRate;
            v.Trim
            ];

        M = subs(M,x,x0);
        H = subs(H,x,x0);
        G = subs(G,x,x0);
    end

    if obj.StateVector.m > 0
        A = obj.Constraints.Jacobian;
        Ad = obj.Constraints.JacobianRate;
        eoma = A*diff(u.All,t) + Ad*u.All;

        M = [
            M;
            subs(jacobian(eoma,diff(X.All,t)),x,x0)
            ];

        H = [
            H;
            subs(jacobian(-eoma,X.All),x,x0)
            ];

        G = [
            G;
            zeros(size(eoma,1),size(G,2))
            ];

        [C1,C2] = dependentVelocityProjection(q,u,obj.Constraints);
        C1 = subs(C1,x,x0);
        C2 = subs(C2,x,x0);
        Hq = H(:,1:obj.StateVector.n);
        Hu = H(:,obj.StateVector.n + 1:end);
        H = [Hq + Hu*C1,Hu*C2];
    end

    if obj.StateVector.l > 0
        C0 = dependentCoordinateProjection(q,obj.Constraints);
        C0 = subs(C0,x,x0);
        Hq = H(:,1:obj.StateVector.n);
        Hu = H(:,obj.StateVector.n + 1:end);
        H = [Hq*C0,Hu];
    end

    eom_lin = LinearizedMotionEquations(X,M,H,G,eomd.Inputs);
end