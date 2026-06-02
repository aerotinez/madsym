classdef NewtonEulerEquations
    properties (GetAccess = public, SetAccess = protected)
        Pose;
        SpatialInertia;
        Twist;
        TwistRate;
        ActiveForces;
        IsTrimmed;
    end
    methods (Access = public)
        function obj = NewtonEulerEquations(body)
            arguments
                body (1,1) Body;
            end
            T = Pose(body.ReferenceFrame,body.MassCenter);
            obj.Pose = T.transform();
            obj.SpatialInertia = body.inertiaMatrix();
            obj.Twist = body.Twist.vector();
            obj.TwistRate = body.Twist.rateVector();
            obj.ActiveForces = body.ActiveForces.vector(T);
            obj.IsTrimmed = false;
        end
    end
end