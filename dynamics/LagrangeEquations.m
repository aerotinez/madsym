classdef LagrangeEquations < EquationsOfMotion
properties (GetAccess = public, SetAccess = private)
    Multipliers (:,1) sym = sym.empty;
    ConstraintForces (:,1) sym = sym.empty;
end
methods (Access = public)
function obj = LagrangeEquations(eomk,bodies,friction_coeffs)
    arguments
        eomk (1,1) KinematicEquations;
        bodies (1,:) Body;
        friction_coeffs sym = sym(0);
    end
    obj@EquationsOfMotion(eomk,bodies,friction_coeffs);
    obj.Multipliers = dynvars('lambda',1:obj.eomk.m);
end
function eomd = solveMultiplers(obj)  
    sol = obj.MassMatrix\obj.ForcingVector;
    obj.ConstraintForces = simplify(expand(sol(end-obj.eomk.m+1:end)));

    x = [
        obj.eomk.qdd;
        obj.ConstraintForces
        ];

    M = obj.MassMatrix(1:obj.eomk.n,1:obj.eomk.n);
    f = obj.ForcingVector(1:obj.eomk.n);

    eom = simplify(expand([M,obj.eomk.A.']*x - f));
    
    eomd = EquationsOfMotion(obj.eomk,obj.bodies);
    eomd.MassMatrix = jacobian(eom,obj.eomk.qdd);
    eomd.ForcingVector = -subs(eom,obj.eomk.qdd,0.*obj.eomk.qdd);
    eomd.FrictionForces = obj.FrictionForces(1:obj.eomk.n);
    eomd.ActiveForces = obj.ActiveForces(1:obj.eomk.n);
    eomd.InertialForces = eomd.ForcingVector - eomd.FrictionForces - eomd.ActiveForces;
end
end
end