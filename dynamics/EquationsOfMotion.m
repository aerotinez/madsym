classdef EquationsOfMotion
properties (GetAccess = public, SetAccess = private)
    Kinematics (1,1) struct;
    Dynamics (1,1) struct;
    Auxiliary (1,1) struct;
end
methods (Access = public)
function obj = EquationsOfMotion(kinematics,dynamics,auxiliary)
    arguments
        kinematics (1,1) KinematicEquations;
        dynamics (1,1) DynamicEquations;
        auxiliary (1,1) AuxiliaryEquations = AuxiliaryEquations();
    end
    eq = @(x,M,f)struct("Variable",x,"MassMatrix",M,"ForcingVector",f);
    q = kinematics.Coordinates;
    u = kinematics.QuasiVelocities;
    Ju = kinematics.QuasiVelocityJacobian(:,1:numel(u));
    obj.Kinematics = eq(q,eye(size(q.All,1)),Ju*[u.Velocity].');
    hc = kinematics.HolonomicConstraints;
    obj.Kinematics.HolonomicConstraints = hc;
    obj.Dynamics = eq(u,dynamics.MassMatrix,dynamics.ForcingVector);
    v = auxiliary.Variables;
    obj.Auxiliary = eq(v,auxiliary.MassMatrix,auxiliary.ForcingVector);
end
end
end