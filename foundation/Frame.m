classdef Frame
properties (GetAccess = public, SetAccess = private)
    x;
    y;
    z;
    dcm;
    w;
    wd; 
end
methods (Access = public)
function obj = Frame(R)
    arguments
        R (3,3) sym = eye(3,'sym');
    end 
    obj.x = R(:,1);
    obj.y = R(:,2);
    obj.z = R(:,3);
    obj.dcm = R;
    obj.w = simplify(expand(skew2vec(obj.dcm.'*diff(obj.dcm))));
    obj.wd = simplify(expand(diff(obj.w)));
end
function Nnew = orientNew(obj,axis,angle)
    arguments
        obj (1,1) Frame;
        axis (1,:) char;
        angle (1,:) sym;
    end
    if ~(length(axis) == length(angle))
        error('axis and angle must be the same length');
    end
    R = obj.dcm;
    for i = 1:length(axis)
        switch axis(i)
        case 'x'
            R = R*Rx(angle(i));
        case 'y'
            R = R*Ry(angle(i));
        case 'z'
            R = R*Rz(angle(i));
        end
    end
    Nnew = Frame(simplify(expand(R)));
end
function w = angVel(obj,N)
    arguments
        obj (1,1) Frame;
        N (1,1) Frame = Frame();
    end 
    w = simplify(expand(N.dcm.'*obj.dcm*obj.w));
end
function wd = angAccel(obj,N)
    arguments
        obj (1,1) Frame;
        N (1,1) Frame = Frame();
    end 
    wd = simplify(expand(N.dcm.'*obj.dcm*obj.wd));
end
function J = jacobian(obj,qd,N)
    arguments
        obj (1,1) Frame;
        qd (1,:) sym;
        N (1,1) Frame = Frame();
    end 
    J = N.dcm*jacobian(obj.w,qd);
end
function Jd = jacobianRate(obj,qd,N) 
    Jd = simplify(expand(diff(obj.jacobian(qd,N))));
end
end
end