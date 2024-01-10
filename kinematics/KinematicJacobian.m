classdef KinematicJacobian
    properties (GetAccess = public, SetAccess = private)
        qd;
        u;
    end
    methods (Access = public)
        function obj = KinematicJacobian(equations,qd)
            obj.qd = jacobian(equations,qd);
            if rank(obj.qd) < size(obj.qd,1)
                msg1 = "Kinematics are non-invertible.";
                error(msg1);
            end
            % obj.qd = simplify(expand(obj.qd));
            % obj.u = simplify(expand(syminv(obj.qd)));
            obj.u = syminv(obj.qd);
        end
    end
end