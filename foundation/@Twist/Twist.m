classdef Twist
    properties (Access = private)
        Pose;
        Vector;
        RateVector; 
    end
    methods
        function obj = Twist(pose)
            arguments
                pose (1,1) Pose = Pose();
            end
            t = sym('t');
            obj.Pose = pose;
            R = obj.Pose.ReferenceFrame.dcm();
            p = obj.Pose.Position.posFrom();
            w = skew2vec(R.'*diff(R,t));
            v = R.'*diff(p,t);
            obj.Vector = [w;v];
            obj.RateVector = diff(obj.Vector,t);
        end 
    end
end