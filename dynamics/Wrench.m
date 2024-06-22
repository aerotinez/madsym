classdef Wrench
properties (GetAccess = public, SetAccess = private)
    ReferenceFrame (1,1) Frame = Frame();
    Position (1,1) Point = Point();
    Vector (6,1) sym = zeros(6,1,'sym');
end
methods (Access = public)
function obj = Wrench(reference_frame,position,vector)
    arguments
        reference_frame (1,1) Frame;
        position (1,1) Point;
        vector (6,1) sym;
    end
    obj.ReferenceFrame = reference_frame;
    obj.Position = position;
    obj.Vector = vector;
end
function new_wrench = transform(obj,reference_frame,position)
    arguments
        obj (1,1) Wrench;
        reference_frame (1,1) Frame;
        position (1,1) Point;
    end
    T = Pose(obj.ReferenceFrame,obj.Position);
    Tnew = Pose(reference_frame,position);
    W = simplify(expand(Tnew.Adjoint.'*T.inv().Adjoint.'))*obj.Vector;
    new_wrench = Wrench(reference_frame,position,W);
end
end
end