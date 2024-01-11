classdef Pose
properties (GetAccess = public, SetAccess = private)
    ReferenceFrame Frame;
    Position Point;
    Transform (4,4) sym = eye(4,'sym');
    Adjoint (6,6) sym = eye(6,'sym');
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
function inv_pose = inv(obj)
    R = obj.ReferenceFrame.dcm;
    p = obj.Position.posFrom(); 
    inv_pose = Pose(Frame(R.'),Point(simplify(expand(-R.'*p))));
end
end
end