classdef KinematicJacobianRate
    properties (GetAccess = public, SetAccess = private)
        qd;
        u;
    end
    methods (Access = public)
        function obj = KinematicJacobianRate(J,q,qd,f)
            arguments
                J (1,1) KinematicJacobian;
                q (:,1) sym;
                qd (:,1) sym;
                f (:,1) sym;
            end
            obj.qd = jacobianRate(J.qd,q,qd);
            obj.u = jacobianRate(J.u,q,f);
        end
    end
end