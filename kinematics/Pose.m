classdef Pose
properties (GetAccess = public, SetAccess = private)
    N Frame;
    P Point;
    T (4,4) sym;
    Ad (6,6) sym;
end
methods
function obj = Pose(N,P)
    arguments
        N (1,1) Frame = Frame();
        P (1,1) Point = Point();
    end
    obj.N = N;
    obj.P = P;
    obj.T = [
        N.dcm,P.posFrom();
        0,0,0,1
        ];
    obj.Ad = [
        N.dcm, zeros(3,3);
        vec2skew(P.posFrom()), N.dcm
        ];
end
function inv_pose = inv(obj)
    Ni = Frame(obj.N.dcm');
    Pi = Point(simplify(expand(-Ni.dcm*obj.P.posFrom())));
    inv_pose = Pose(Ni,Pi);
end
end
end