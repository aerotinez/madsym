classdef Body < handle
    properties (GetAccess = public, SetAccess = private)
        ReferenceFrame;
        MassCenter;
        Twist; 
        Inertia;
        Mass; 
        ActiveForces;
    end
    methods
        function obj = Body(reference_frame,mass_center,inertia,mass)
            arguments
                reference_frame (1,1) Frame = Frame();
                mass_center (1,1) Point = Point();
                inertia (3,3) sym = zeros(3,3,'sym');
                mass (1,1) sym = sym(0);
            end
            obj.ReferenceFrame = reference_frame;
            obj.Twist = Twist(Pose(reference_frame,mass_center));
            obj.MassCenter = mass_center;
            obj.Inertia = inertia;
            obj.Mass = mass;
            obj.ActiveForces = Wrench();
        end
    end
end