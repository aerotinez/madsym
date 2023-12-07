classdef Twist
properties (GetAccess = public, SetAccess = private)
    AngularVelocity (3,1) sym = zeros(3,1,'sym');
    TranslationalVelocity (3,1) sym = zeros(3,1,'sym');
    Vector (6,1) sym = zeros(6,1,'sym');
    Matrix (4,4) sym = zeros(4,4,'sym');
    Adjoint (6,6) sym = zeros(6,6,'sym');
    Rate (6,1) sym = zeros(6,1,'sym');
end
methods
function obj = Twist(pose)
    arguments
        pose (1,1) Pose = Pose();
    end
    obj.Matrix = simplify(expand(pose.inv().Matrix*diff(pose.Matrix,sym('t'))));
    obj.AngularVelocity = skew2vec(obj.Matrix(1:3,1:3));
    obj.TranslationalVelocity = obj.Matrix(1:3,4);
    obj.Vector = [
        obj.AngularVelocity;
        obj.TranslationalVelocity
        ];
    obj.Adjoint = [
        vec2skew(obj.AngularVelocity),zeros(3);
        vec2skew(obj.TranslationalVelocity),vec2skew(obj.AngularVelocity)
        ];
    obj.Rate = simplify(expand(diff(obj.Vector,sym('t'))));
end
function J = jacobian(obj,q,pose)
    arguments
        obj (1,1) Twist;
        q (1,1) GeneralizedCoordinates;
        pose (1,1) Pose = Pose();
    end
    J = simplify(expand(pose.Adjoint*jacobian(obj.Vector,q.Rates)));
end
function Jd = jacobianRate(obj,q,pose)
    arguments
        obj (1,1) Twist;
        q (1,1) GeneralizedCoordinates;
        pose (1,1) Pose = Pose();
    end
    Jd = simplify(expand(diff(obj.jacobian(q.Rates,pose),sym('t'))));
end
end
end