classdef NewtonEulerEquations
    properties (GetAccess = public, SetAccess = protected)
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
            T = Pose(Frame(),Point());
            obj.SpatialInertia = body.inertiaMatrix(T);
            obj.Twist = body.Twist.vector(T);
            obj.TwistRate = body.Twist.rateVector(T);
            obj.ActiveForces = body.ActiveForces.vector(T);
            obj.IsTrimmed = false;
        end
    end
end