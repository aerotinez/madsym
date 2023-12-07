classdef Body < handle
properties (GetAccess = public, SetAccess = private)
    ReferenceFrame (1,1) Frame;
    MassCenter (1,1) Point; 
    Inertia (3,3) sym = zeros(3,3,'sym');
    Mass (1,1) sym = sym(0); 
    InertialForces (:,1) sym;
    ActiveForces (:,1) sym;
end
methods
function obj = Body(reference_frame,mass_center,inertia,mass)
    arguments
        reference_frame (1,1) Frame = Frame();
        mass_center (1,1) Point = Point();
        inertia (3,3) sym = zeros(3,3,'sym');
        mass (1,1) sym = sym(0);
    end
    obj.ReferenceFrame = reference_frame;
    obj.MassCenter = mass_center;
    obj.Inertia = inertia;
    obj.Mass = mass;
    obj.InertialForces = obj.inertialForces();
    obj.ActiveForces = sym(zeros(6,1));
end
function applyForce(obj,frame,point,force)
    arguments
        obj (1,1) Body;
        frame (1,1) Frame;
        point (1,1) Point;
        force (3,1) sym;
    end
    obj.applyWrench(frame,point,[0;0;0;force]);
end
function applyMoment(obj,frame,moment)
    arguments
        obj (1,1) Body;
        frame (1,1) Frame;
        moment (3,1) sym;
    end
    obj.applyWrench(frame,Point(obj.p),[moment;0;0;0]);
end
end
methods (Access = private)
function Qi = inertialForces(obj)
    G = blkdiag(obj.Inertia,obj.Mass*eye(3));
    twist = Twist(Pose(obj.ReferenceFrame,obj.MassCenter));
    V = twist.Vector;
    ad = twist.Adjoint;
    Vd = twist.Rate;
    Qi = simplify(expand(G*Vd - ad.'*G*V));
end
function W = applyWrench(obj,frame,point,twist)
    Adspace = Pose(frame,point).inv.Adjoint;
    Adbody = Pose(obj.ReferenceFrame,obj.MassCenter).Adjoint;
    W = simplify(expand(Adbody*Adspace.'*twist));
    obj.ActiveForces = simplify(expand(obj.ActiveForces + W));
end
end
end