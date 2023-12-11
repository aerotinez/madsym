classdef GibbsAppellDynamics < DynamicsStrategy
    properties (GetAccess = public, SetAccess = private)
        q;
        u;
        v;
        Vbar;
        Vdbar;
        G;
        F;
    end
    methods (Access = public)
        function obj = GibbsAppellDynamics(kinematics,G,J,F,q,u,v)
            arguments
                kinematics (1,1) KinematicEquations;
                G sym;
                J sym;
                F sym;
                q (:,1) sym;
                u (:,1) sym;
                v (:,1) sym = sym.empty(0,1); 
            end
            obj.q = q;
            obj.u = u;
            obj.v = v;
            obj.Vbar = simplify(expand(J*kinematics.Jacobian.u));
            fk = kinematics.ForcingVector;
            obj.Vdbar = simplify(expand(jacobianRate(Vbar,q,fk)));
            obj.G = G;
            obj.F = F;
        end
        function eomd = dynamicEquations(obj)
        end
    end
end