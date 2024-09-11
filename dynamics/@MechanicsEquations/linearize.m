function eom_lin = linearize(obj,trim_point)
    arguments
        obj (1,1) MechanicsEquations;
        trim_point (1,1) TrimPoint = TrimPoint(obj.StateVector,obj.Inputs);
    end

    eomk = linearize(obj.Kinematics,trim_point);
    eomd = sum(arrayfun(@(eom)linearize(eom,trim_point),obj.BodyDynamics));

    x = [
        trim_point.q;
        trim_point.u;
        ];

    u = trim_point.F;

    M = [
        eomk.MassMatrix;
        eomd.MassMatrix;
        ];

    H = [
        eomk.ForcingMatrix;
        eomd.ForcingMatrix;
        ];

    G = [
        zeros(size(eomk.ForcingMatrix,1),size(eomd.InputMatrix,2));
        eomd.InputMatrix;
        ];

    X = obj.StateVector;

    if obj.StateVector.m > 0
        Ma = obj.Constraints.Acceleration.MassMatrix;
        fa = obj.Constraints.Acceleration.ForcingVector;
        eoma = Ma*obj.Constraints.Acceleration.Rates - fa;
        xd = diff(x);

        M = [
            M;
            subsTrim(jacobian(eoma,xd),trim_point)
            ];

        H = [
            H;
            subsTrim(jacobian(-eoma,x),trim_point)
            ];

        G = [
            G;
            zeros(size(eoma,1),size(G,2))
            ];

        [C1,C2] = dependentVelocityProjection(X,obj.Constraints);
        C1 = subsTrim(C1,trim_point);
        C2 = subsTrim(C2,trim_point);
        Hq = H(:,1:obj.StateVector.n);
        Hu = H(:,obj.StateVector.n + 1:end);
        H = [Hq + Hu*C1,Hu*C2];
    end

    if obj.StateVector.l > 0
        q = X.Coordinates;
        C0 = dependentCoordinateProjection(q,obj.Constraints);
        C0 = subsTrim(C0,trim_point);
        Hq = H(:,1:obj.StateVector.n);
        Hu = H(:,obj.StateVector.n + 1:end);
        H = [Hq*C0,Hu];
    end

    eom_lin = LinearizedMotionEquations(x,M,H,G,u);
end