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

    eom_lin = LinearizedMotionEquations(x,M,H,G,u);
end