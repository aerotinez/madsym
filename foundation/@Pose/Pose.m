classdef Pose
    properties (GetAccess = public, SetAccess = private)
        ReferenceFrame;
        Position;
        Transform;
        Adjoint;
    end
    methods (Access = public)
        function obj = Pose(reference_frame,position)
            arguments
                reference_frame (1,1) Frame = Frame();
                position (1,1) Point = Point();
            end
            obj.ReferenceFrame = reference_frame;
            obj.Position = position;

            R = obj.ReferenceFrame.dcm;
            p = obj.Position.posFrom();

            obj.Transform = [
                R,p;
                0,0,0,1
                ];

            obj.Adjoint = [
                R, zeros(3,3);
                vec2skew(p)*R,R
                ];
        end 
    end
end