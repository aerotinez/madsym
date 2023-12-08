classdef KinematicEquations
properties (GetAccess = public, SetAccess = private)
    Coordinates GeneralizedCoordinates;
    QuasiVelocities QuasiVariable;
    HolonomicConstraints (:,1) sym = sym.empty();
    NonholonomicConstraints (:,1) sym = sym.empty();
    KinematicJacobian sym;
    ConstraintJacobian sym;
    GeneralizedVelocityJacobian sym;
    QuasiVelocityJacobian sym;
    GeneralizedVelocityJacobianRate sym;
    QuasiVelocityJacobianRate sym;
end
methods (Access = public)
function obj = KinematicEquations(coordinates,quasi_velocities,equations,options)
    arguments
        coordinates (1,1) GeneralizedCoordinates = GeneralizedCoordinates();
        quasi_velocities (:,1) QuasiVariable = QuasiVariable();
        equations (:,1) sym = sym.empty([0,1]);
        options.HolonomicConstraints (:,1) sym = sym.empty();
        options.NonholonomicConstraints (:,1) sym = sym.empty();
    end
    obj.Coordinates = coordinates;
    obj.QuasiVelocities = quasi_velocities;
    qd = [coordinates.All.Velocity].';
    u = [quasi_velocities.Velocity].';
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
    obj.GeneralizedVelocityJacobian = [
        obj.KinematicJacobian;
        obj.ConstraintJacobian
    ];
    Ju = obj.GeneralizedVelocityJacobian\eye(numel(qd));
    obj.QuasiVelocityJacobian = simplify(expand(Ju));
    Jdqd = diff(obj.GeneralizedVelocityJacobian,sym('t'));
    obj.GeneralizedVelocityJacobianRate = simplify(expand(Jdqd));
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