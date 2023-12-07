classdef KinematicEquations
properties (GetAccess = public, SetAccess = private)
    HolonomicConstraints (:,1) sym = sym.empty();
    NonholonomicConstraints (:,1) sym = sym.empty();
    KinematicJacobian sym;
    ConstraintJacobian sym;
    GeneralizedRateJacobian sym;
    QuasiVelocityJacobian sym;
    GeneralizedRateJacobianRate sym;
    QuasiVelocityJacobianRate sym;
end
methods (Access = public)
function obj = KinematicEquations(coordinates,velocities,equations,options)
    arguments
        coordinates (1,1) GeneralizedCoordinates;
        velocities (1,1) Velocities;
        equations (:,1) sym;
        options.HolonomicConstraints (:,1) sym = sym.empty();
        options.NonholonomicConstraints (:,1) sym = sym.empty();
    end
    qd = coordinates.Rates;
    u = velocities.Quasi.States;
    hc = options.HolonomicConstraints;
    nhc = options.NonholonomicConstraints;
    obj.validateCoodinateDimensions(qd,u,hc,nhc);
    obj.validateKinematics(qd,u,hc,nhc,equations);
    obj.HolonomicConstraints = simplify(expand(hc));
    obj.NonholonomicConstraints = simplify(expand(nhc));
    Jk = jacobian(u - equations,qd);
    obj.KinematicJacobian = simplify(expand(Jk));
    c = [
        diff(obj.HolonomicConstraints,sym('t'));
        obj.NonholonomicConstraints
    ];
    obj.ConstraintJacobian = simplify(expand(jacobian(c,qd)));
    obj.GeneralizedRateJacobian = [
        obj.KinematicJacobian;
        obj.ConstraintJacobian
    ];
    Ju = obj.GeneralizedRateJacobian\eye(numel(qd));
    obj.QuasiVelocityJacobian = simplify(expand(Ju));
    Jdqd = diff(obj.GeneralizedRateJacobian,sym('t'));
    obj.GeneralizedRateJacobianRate = simplify(expand(Jdqd));
    qds = simplify(expand(Ju(:,1:numel(u))*u));
    Jdu = -Ju*subs(Jdqd,qd,qds)*Ju;
    obj.QuasiVelocityJacobianRate = simplify(expand(Jdu));
end
end
methods (Access = private)
function validateCoodinateDimensions(~,qd,u,hc,nhc)
    n = numel(qd);
    k = numel(u);
    m = size([hc;nhc],1); 
    if n ~= k + m
        msga = "Number of coordinates must equal";
        msgb = " number of quasi-velocities + constraints";
        error(strcat(msga,msgb)); 
    end
end
function validateKinematics(~,qd,u,hc,nhc,eq)
    sys = [
        u - eq;
        diff(hc,sym('t'));
        nhc
    ];
    Jqd = jacobian(sys,qd);
    if rank(Jqd) < size(Jqd,1)
        msg = "Kinematics are not invertible";
        error(msg);
    end
end
end
end