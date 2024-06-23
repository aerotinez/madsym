classdef Wrench
    properties (GetAccess = public, SetAccess = private)
        ReferenceFrame;
        Position;
        Vector;
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
    end
end