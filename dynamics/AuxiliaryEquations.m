classdef AuxiliaryEquations
properties (GetAccess = public, SetAccess = private)
    Variables (:,1) QuasiVariable = QuasiVariable.empty([0,1]);
    MassMatrix sym = sym.empty();
    ForcingVector sym = sym.empty();
end
methods (Access = public)
function obj = AuxiliaryEquations(variables,equations,kinematic_equations)
    arguments
        variables (:,1) QuasiVariable = QuasiVariable.empty([0,1]);
        equations (:,1) sym = sym.empty([0,1]);
        kinematic_equations KinematicEquations {mustBeScalarOrEmpty} = KinematicEquations.empty();
    end
    obj.Variables = variables;
    vd = [obj.Variables.Acceleration].';
    if ~isempty(kinematic_equations)
        qd = [kinematic_equations.Coordinates.All.Velocity].';
        u = [kinematic_equations.QuasiVelocities.Velocity].';
        Ju = kinematic_equations.QuasiVelocityJacobian(:,1:numel(u));
        qds = Ju*u;
        eom = subs(equations,qd,qds);
    else
        eom = equations;
    end
    obj.MassMatrix = jacobian(eom,vd);
    obj.ForcingVector = -subs(eom,vd,0.*vd);
end
end
end