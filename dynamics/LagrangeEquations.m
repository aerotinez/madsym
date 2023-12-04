classdef LagrangeEquations < EquationsOfMotion
properties (GetAccess = public, SetAccess = private)
    Multipliers (:,1) sym = sym.empty;
    ConstraintForces (:,1) sym = sym.empty;
end
methods (Access = public)
function obj = LagrangeEquations(eomk,bodies,options)
    arguments
        eomk (1,1) KinematicEquations;
        bodies (1,:) Body;
        options.Inputs sym = sym.empty;
        options.FrictionCoeffs sym = zeros([numel(eomk.q),1],'sym');
    end
    obj@EquationsOfMotion(eomk,bodies, ...
        'Inputs',options.Inputs, ...
        'FrictionCoeffs',options.FrictionCoeffs);
    obj.Multipliers = dynvars('lambda',1:obj.eomk.m);
end
function eomd = solveMultipliers(obj)  
    sol = obj.MassMatrix\obj.ForcingVector;
    lm = simplify(expand(sol(end-obj.eomk.m+1:end)));
    obj.ConstraintForces = simplify(expand(obj.eomk.A.'*lm));
    qdd = obj.eomk.qdd;

    x = [
        qdd;
        lm
        ];

    M = obj.MassMatrix(1:obj.eomk.n,:);
    f = obj.ForcingVector(1:obj.eomk.n);

    eom = simplify(expand(M*x - f));
    
    eomd = EquationsOfMotion(obj.eomk,obj.bodies);
    eomd.MassMatrix = jacobian(eom,qdd);
    eomd.ForcingVector = -subs(eom,qdd,0.*qdd);
    eomd.FrictionForces = obj.FrictionForces(1:obj.eomk.n);
    eomd.ActiveForces = obj.ActiveForces(1:obj.eomk.n);
    eomd.InertialForces = eomd.ForcingVector - eomd.FrictionForces - eomd.ActiveForces;
end
end
end