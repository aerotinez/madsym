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
            w = simplify(expand(skew2vec(R.'*diff(R,t))));
            v = simplify(expand(R.'*diff(p,t)));
            obj.Vector = [w;v];
            obj.RateVector = simplify(expand(diff(obj.Vector,t)));
        end 
    end
end