classdef Body < handle
properties (GetAccess = public, SetAccess = private)
    q (:,1) sym;
    qd (:,1) sym;
    R (3,3) sym;
    p (3,1) sym;
    V (6,1) sym;
    J (6,:) sym;
    ad (6,6) sym;
    Vd (6,1) sym;
    Jd (6,:) sym;
    m (1,1) sym;
    I (3,3) sym;
    G (6,6) sym;
    L (1,1) sym;
    M sym;
    Q (:,1) sym;
end
methods
function obj = Body(q,pose,mass,inertia)
    arguments
        q (:,1) sym;
        pose (1,1) Pose = Pose();
        mass (1,1) sym = sym(0);
        inertia (3,3) sym = zeros(3,3,'sym');
    end
    obj.q = q;
    obj.qd = diff(q);
    obj.R = pose.dcm;
    obj.p = pose.P.posFrom();
    twist = Twist(pose);
    obj.V = twist.V;
    obj.J = twist.jacobian(obj.qd);
    obj.ad = twist.ad;
    obj.Vd = twist.Vd;
    obj.Jd = twist.jacobian(obj.qd);
    obj.m = mass;
    obj.I = inertia;
    obj.G = blkdiag(obj.I,obj.m*eye(3));
    obj.L = simplify(expand((1/2)*obj.V.'*obj.G*obj.V));
    obj.M = simplify(expand(obj.J.'*obj.G*obj.J));
    obj.Q = zeros(size(obj.q));
end
function applyForce(obj,frame,point,force,reacting_body)  
    obj.applyWrench(frame,point,[0;0;0;force],reacting_body);
end
function applyMoment(obj,frame,moment,reacting_body) 
    obj.applyWrench(frame,Point(obj.p),[moment;0;0;0],reacting_body);
end
end
methods (Access = private)
function W = wrench(~,frame,point,twist)
    W = simplify(expand(Pose(frame,point).inv.Ad.'*twist)); 
end
function applyWrench(obj,frame,point,wrench,reacting_body)
    arguments
        obj (1,1) Body;
        frame (1,1) Frame;
        point (1,1) Point;
        wrench (6,1) sym;
        reacting_body (1,1) Body = Body(obj.q);
    end
    w = obj.wrench(frame,point,wrench);
    fJ = @(b)simplify(expand(Pose(Frame(b.R),Point(b.p)).Ad*b.J));
    obj.Q = simplify(expand(obj.Q + fJ(obj).'*w - fJ(reacting_body).'*w));
end
end
end