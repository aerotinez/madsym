classdef Point
properties (GetAccess = public, SetAccess = private)
    x (1,1) sym = sym(0);
    y (1,1) sym = sym(0); 
    z (1,1) sym = sym(0);
end
methods
function obj = Point(p)
    arguments
        p (3,1) sym = zeros(3,1,'sym');
    end
    obj.x = p(1);
    obj.y = p(2);
    obj.z = p(3);
end
function P = locateNew(obj,p)
    arguments
        obj (1,1) Point;
        p (3,1) sym = zeros(3,1,'sym');
    end
    P = Point(simplify(expand([obj.x;obj.y;obj.z] + p)));
end
function d = posFrom(obj,P,N)
    arguments
        obj (1,1) Point;
        P (1,1) Point = Point();
        N (1,1) Frame = Frame();
    end
    d = simplify(expand(N.dcm.'*([obj.x;obj.y;obj.z] - [P.x;P.y;P.z])));
end
end
end