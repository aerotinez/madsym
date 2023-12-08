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
    V = pose.inv().Transform*diff(pose.Transform,sym('t'));
    obj.Matrix = simplify(expand(V));
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
function J = jacobian(obj,qd,pose)
    arguments
        obj (1,1) Twist;
        qd (:,1) sym;
        pose (1,1) Pose = Pose();
    end
    J = simplify(expand(pose.Adjoint*jacobian(obj.Vector,qd)));
end
function Jd = jacobianRate(obj,qd,pose)
    arguments
        obj (1,1) Twist;
        qd (:,1) sym;
        pose (1,1) Pose = Pose();
    end
    Jd = simplify(expand(diff(obj.jacobian(qd,pose),sym('t'))));
end
end
end