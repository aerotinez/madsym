classdef TrimPoint
properties (GetAccess = public, SetAccess = private)
    q (:,1) sym = sym.empty([0,1]);
    u (:,1) sym = sym.empty([0,1]);
    ud (:,1) sym = sym.empty([0,1]);
    v (:,1) sym = sym.empty([0,1]);
    vd (:,1) sym = sym.empty([0,1]);
    f (:,1) sym = sym.empty([0,1]);
end
properties (GetAccess = public, SetAccess = private)
    q0 (:,1) sym = sym.empty([0,1]);
    u0 (:,1) sym = sym.empty([0,1]);
    ud0 (:,1) sym = sym.empty([0,1]);
    v0 (:,1) sym = sym.empty([0,1]);
    vd0 (:,1) sym = sym.empty([0,1]);
    f0 (:,1) sym = sym.empty([0,1]);
end
methods
function obj = TrimPoint(equations,inputs,q0,u0,v0,f0)
    arguments
        equations EquationsOfMotion {mustBeScalarOrEmpty} = MechanicalEquations.empty([0,1]);
        inputs (:,1) sym = sym.empty([0,1]);
        q0 (:,1) sym = sym.empty([0,1]);
        u0 (:,1) sym = sym.empty([0,1]);
        v0 (:,1) sym = sym.empty([0,1]);
        f0 (:,1) sym = sym.empty([0,1]);
    end
    obj.q = [equations.Kinematics.Variable.All.Position].';
    obj.u = [equations.Dynamics.Variable.Velocity].';
    obj.ud = [equations.Dynamics.Variable.Acceleration].';
    obj.v = [equations.Auxiliary.Variable.Velocity].';
    obj.vd = [equations.Auxiliary.Variable.Acceleration].';
    obj.f = inputs;
    obj.q0 = q0;
    obj.u0 = u0;
    obj.ud0 = 0.*u0;
    obj.v0 = v0;
    obj.vd0 = zeros(size(v0),'sym');
    obj.f0 = f0;
end
end
end