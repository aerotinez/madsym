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
    eomaux = obj.AuxilaryEquations;
    p = numel(obj.AuxilarySpeeds);

    x = [
        qdd;
        lm
        ];

    M = obj.MassMatrix(1:obj.eomk.n,:);
    f = obj.ForcingVector(1:obj.eomk.n);

    eom = simplify(expand(M*x - f));
    
    eomd = EquationsOfMotion(obj.eomk,obj.bodies);

    Maux = jacobian(eomaux,diff(obj.AuxilarySpeeds));
    eomd.MassMatrix = [
        jacobian(eom,qdd),zeros(obj.eomk.n,p);
        zeros(p,obj.eomk.n),Maux
        ];

    eomd.ForcingVector = [
        -subs(eom,qdd,0.*qdd);
        -subs(eomaux,diff(obj.AuxilarySpeeds),0.*obj.AuxilarySpeeds)
        ];
    
    if ~isempty(symvar(obj.FrictionCoefficients))
        Qf = jacobian(eom,obj.FrictionCoefficients)*obj.FrictionCoefficients;
        eomd.FrictionForces = Qf;
    end
    eomd.ActiveForces = jacobian(eom,obj.Inputs)*obj.Inputs;
    Qi = eomd.ForcingVector - eomd.ActiveForces;
    eomd.InertialForces = simplify(expand(Qi));
end
end
end