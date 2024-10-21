classdef Wrench
    properties (Access = private)
        Vector;
    end
    methods (Access = public)
        function obj = Wrench(vector,pose)
            arguments
                vector (6,1) sym = zeros(6,1,"sym");
                pose (1,1) Pose = Pose();
            end
            obj.Vector = pose.inv().adjoint().'*vector;
        end
    end
end