classdef EquationsOfMotion < handle 
properties (Access = public)
    eomk KinematicEquations = KinematicEquations.empty;
    bodies (:,1) Body;
    Inputs (:,1) sym;
    AuxiliarySpeeds (:,1) sym;
    AuxiliaryEquations (:,1) sym; 
    Lagrangian (1,1) sym = sym(0);
    SpatialInertia sym;
    FrictionCoefficients sym;
    MassMatrix sym;
    Jacobian sym;
    JacobianRate sym;
    TwistAdjoint sym;
    ForcingVector sym;
    InertialForces sym;
    ActiveForces sym;
    FrictionForces sym;
end
methods (Access = public)
function obj = EquationsOfMotion(eomk,bodies,options)
    arguments
        eomk (1,1) KinematicEquations;
        bodies (:,1) Body;
        options.Inputs (:,1) sym = sym.empty;
        options.FrictionCoeffs sym = zeros([numel(eomk.q),1],'sym');
        options.AuxiliarySpeeds (:,1) sym = sym.empty;
        options.AuxiliaryEquations (:,1) sym = sym.empty;
    end
    obj.eomk = eomk;
    obj.bodies = bodies;
    obj.Inputs = options.Inputs;
    obj.AuxiliarySpeeds = options.AuxiliarySpeeds;
    obj.AuxiliaryEquations = options.AuxiliaryEquations;
    obj.Lagrangian = obj.mapsum(@(b)b.L,1);
    obj.SpatialInertia = obj.blkdiag(@(b)b.G);
    obj.Jacobian = cell2sym(arrayfun(@(b)b.J,bodies,'uniform',0));
    obj.MassMatrix = obj.Jacobian.'*obj.SpatialInertia*obj.Jacobian;
    obj.MassMatrix = simplify(expand(obj.MassMatrix));
    obj.JacobianRate = cell2sym(arrayfun(@(b)b.Jd,bodies,'uniform',0));
    obj.TwistAdjoint = obj.blkdiag(@(b)b.ad);
    obj.ActiveForces = obj.mapsum(@(b)b.Q,2);
    obj.FrictionCoefficients = options.FrictionCoeffs; 
    obj.FrictionForces = -diag(obj.FrictionCoefficients)*obj.eomk.qd;
end
function eomd = derive(obj,f)
    arguments
        obj (1,1) EquationsOfMotion;
        f (1,1) function_handle;
    end
    eomd = f(obj); 
end
end
methods (Access = private)
function out = mapsum(obj,f,n)
    arguments
        obj (1,1) EquationsOfMotion;
        f (1,1) function_handle;
        n (1,1) double;
    end
    switch n
    case 1
        out = sum(arrayfun(@(b)f(b),obj.bodies));
    case 2
        out = sum(cell2sym(arrayfun(@(b)f(b),obj.bodies.','uniform',0)),2);
    case 3
        outc = arrayfun(@(b)f(b),obj.bodies,'uniform',0);
        out = sum(cell2sym(reshape(outc,1,1,[])),3);
    otherwise
        error('n must be 1, 2, or 3');
    end
    out = simplify(expand(out));
end
function out = blkdiag(obj,f)
    out = zeros(6*numel(obj.bodies),'sym');
    for i = 1:numel(obj.bodies)
        out(6*(i-1)+(1:6),6*(i-1)+(1:6)) = f(obj.bodies(i));
    end
end
end
end