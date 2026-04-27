classdef DynamicEquations < MotionEquations
    properties (GetAccess = public, SetAccess = protected, Hidden = true)
        SpatialInertia;
        Jacobian;
        JacobianRate;
        ActiveForces;
        ConstraintJacobian;
    end
    methods (Access = public)
        function obj = DynamicEquations(u,G,J,dJ,W,F)
            arguments
                u (:,1) DynamicVariable
                G (6,6) sym
                J (6,:) sym
                dJ (6,:) sym
                W (6,1) sym = zeros(6,1,'sym')
                F (:,1) DynamicVariable = DynamicVariable.empty(0,1)
            end
        
            Jc = eye(numel(u),'sym');
        
            V = J*state(u);
            w = V(1:3);
            wm = vec2skew(w);
            adw = blkdiag(wm,wm);
        
            M = J.'*G*J;
            f = J.'*(W - (G*dJ + adw*G*J)*state(u));
        
            obj@MotionEquations(u,M,f,F);
        
            obj.SpatialInertia = G;
            obj.Jacobian = J;
            obj.JacobianRate  = dJ;
            obj.ActiveForces = W;
            obj.ConstraintJacobian = Jc;
        end
    end
end
