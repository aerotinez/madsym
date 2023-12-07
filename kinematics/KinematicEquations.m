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
        velocities (:,1) Velocities;
        equations (:,1) sym;
        options.HolonomicConstraints (:,1) sym = sym.empty();
        options.NonholonomicConstraints (:,1) sym = sym.empty();
    end
    hc = options.HolonomicConstraints;
    nhc = options.NonholonomicConstraints;
    obj.HolonomicConstraints = simplify(expand(hc));
    obj.NonholonomicConstraints = simplify(expand(nhc));
    sys = [
        velocities.States - equations;
        diff(obj.HolonomicConstraints,sym('t'));
        obj.NonholonomicConstraints
    ];
    obj.GeneralizedRateJacobian = jacobian(sys,coordinates.Rates);
    obj.validate(obj);
    Jqd = simplify(expand(obj.GeneralizedRateJacobian));
    obj.KinematicJacobian = Jqd(1:numel(velocities.Quasi.States),:);
    obj.ConstraintJacobian = Jqd(numel(velocities.Quasi.States) + 1:end,:);
    obj.GeneralizedRateJacobianRate = simplify(expand(diff(Jqd,sym('t'))));
    obj.QuasiVelocityJacobian = Jqd\
end
end
methods (Access = private)
function validate(obj)
    Jqd = obj.GeneralizedRateJacobian;
    if rank(Jqd) < size(Jqd,2)
        error('KinematicJacobian is not full rank');
    end
end
end
end