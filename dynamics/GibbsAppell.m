classdef GibbsAppell
properties (GetAccess = public, SetAccess = private)
    qd sym;
    qdd sym;
    K sym;
    u sym;
    ud sym;
    Ju sym;
    Jdu sym;
    qds sym;
    qdds sym;
    eom sym;
    M sym;
    f sym;
end
methods (Access = public)
function obj = GibbsAppell(kinematic_equations,bodies)
    arguments 
        kinematic_equations (1,1) KinematicEquations;
        bodies (:,1) Body;
    end
    obj.qd = [kinematic_equations.Coordinates.All.Velocity].';
    obj.qdd = [kinematic_equations.Coordinates.All.Acceleration].';
    obj.K = [kinematic_equations.Coordinates.All.Stiffness].';
    obj.u = [kinematic_equations.QuasiVelocities.Velocity].';
    obj.ud = [kinematic_equations.QuasiVelocities.Acceleration].';
    obj.Ju = kinematic_equations.QuasiVelocityJacobian(:,1:numel(obj.u));
    obj.Jdu = kinematic_equations.QuasiVelocityJacobianRate(:,1:numel(obj.u));
    obj.qds = obj.Ju*obj.u;
    obj.qdds = obj.Jdu*obj.u + obj.Ju*obj.ud;

    J = obj.jacobians(bodies);
    Fi = obj.inertialForces(bodies);
    Fa = obj.activeForces(bodies);
    fQ = @(J,F)J.'*F;
    Qi = sum(cell2sym(cellfun(fQ,J,Fi,'UniformOutput',false).'),2);
    Qa = -sum(cell2sym(cellfun(fQ,J,Fa,'UniformOutput',false).'),2);
    Qf = diag(obj.K)*obj.qds;
    obj.eom = obj.Ju.'*(Qi + Qa + Qf);
    obj.M = jacobian(obj.eom,obj.ud);
    obj.f = -subs(obj.eom,obj.ud,zeros(size(obj.ud)));
end
end
methods (Access = private)
function J = jacobians(obj,bodies)
    fJ = @(b)Twist(Pose(b.ReferenceFrame,b.MassCenter)).jacobian(obj.qd);
    J = arrayfun(fJ,bodies,'UniformOutput',false);
end
function Qi = inertialForces(obj,bodies)
    x = [
        obj.qdd;
        obj.qd;
    ];

    xs = [
        obj.qdds;
        obj.qds;
    ];

    Qi = arrayfun(@(b)subs(b.InertialForces,x,xs),bodies,'UniformOutput',false);
end
function Qa = activeForces(~,bodies)
    Qa = arrayfun(@(b)b.ActiveForces,bodies,'UniformOutput',false);
end
end
end