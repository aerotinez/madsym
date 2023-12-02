classdef Twist
properties (GetAccess = public, SetAccess = private)
    w (3,1) sym = zeros(3,1,'sym');
    v (3,1) sym = zeros(3,1,'sym');
    V (6,1) sym = zeros(6,1,'sym');
    Vd (6,1) sym = zeros(6,1,'sym');
    S (4,4) sym = zeros(4,4,'sym');
    ad (6,6) sym = zeros(6,6,'sym'); 
end
methods
function obj = Twist(pose)
    arguments
        pose (1,1) Pose = Pose();
    end
    obj.S = simplify(expand(pose.inv().T*diff(pose.T)));
    obj.w = skew2vec(obj.S(1:3,1:3));
    obj.v = obj.S(1:3,4);

    obj.V = [
        obj.w;
        obj.v
        ];

    obj.Vd = simplify(expand(diff(obj.V)));

    obj.ad = [
        vec2skew(obj.w),zeros(3);
        vec2skew(obj.v),vec2skew(obj.w)
        ];
end
function J = jacobian(obj,qd,pose)
    arguments
        obj (1,1) Twist;
        qd (:,1) sym;
        pose (1,1) Pose = Pose();
    end
    J = simplify(expand(pose.Ad*jacobian(obj.V,qd)));
end
function Jd = jacobianRate(obj,qd,pose)
    Jd = simplify(expand(diff(obj.jacobian(qd,pose))));
end
end
end