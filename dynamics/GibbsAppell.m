classdef GibbsAppell
properties (GetAccess = public, SetAccess = private) 
    Bodies (:,1) GibbsAppellBodyEquations;
    MassMatrix sym;
    ForcingVector sym;
    Linearized LinearizedDynamicEquations = LinearizedDynamicEquations();
end
methods (Access = public)
function obj = GibbsAppell(kinematic_equations,bodies)
    arguments 
        kinematic_equations (1,1) KinematicEquations;
        bodies (:,1) Body;
    end
    q = [kinematic_equations.Coordinates.All.Position].';
    qd = [kinematic_equations.Coordinates.All.Velocity].';
    u = [kinematic_equations.QuasiVelocities.Velocity].';
    ud = [kinematic_equations.QuasiVelocities.Acceleration].';
    Ju = kinematic_equations.Jacobians.Quasi(:,1:numel(u));
    qds = kinematic_equations.ForcingVector;
    Jdu = kinematic_equations.Jacobians.QuasiRate(:,1:numel(u));
    K = [kinematic_equations.Coordinates.All.Stiffness].';
    obj.Bodies = obj.bodies(q,qd,u,ud,Ju,qds,Jdu); 
    obj.MassMatrix = obj.reduce(@(b)b.MassMatrix);
    Qf = Ju.'*diag(K)*qds;
    obj.ForcingVector = obj.reduce(@(b)b.ForcingVector) - Qf;
    obj.Linearized.setMassMatrix(obj.reduce(@(b)b.Linearized.MassMatrix));
    Hfl = -[jacobian(Qf,q),jacobian(Qf,u)];
    Hl = obj.reduce(@(b)b.Linearized.ForcingMatrix) + Hfl;
    obj.Linearized.setForcingMatrix(Hl); 
end
end
methods (Access = private)
function B = bodies(obj,q,qd,u,ud,Ju,qds,Jdu)
    fB = @(b)GibbsAppellBodyEquations(q,qd,u,ud,Ju,qds,Jdu,b);
    B = obj.arrayfun(fB,obj.Bodies);
end
function b = reduce(obj,f)
    b = sum(cell2sym(arrayfun(f,reshape(obj.Bodies,1,1,[]),'uniform',0)),3);
end
end
end