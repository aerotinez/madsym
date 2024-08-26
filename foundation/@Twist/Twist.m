classdef Twist
    properties (Access = private)
        w;
        v;
        wd;
        vd;
    end
    methods
        function obj = Twist(pose)
            arguments
                pose (1,1) Pose = Pose();
            end
            t = sym('t');
            R = pose.ReferenceFrame.dcm();
            p = pose.Position.posFrom();
            f = @(x)simplify(expand(x));
            obj.w = f(skew2vec(diff(R,t)*R.'));
            obj.v = f(diff(p,t) - vec2skew(obj.w)*p);
            obj.wd = f(diff(obj.w,t));
            obj.vd = f(diff(obj.v,t));
        end 
    end
end