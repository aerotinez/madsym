classdef KinematicEquations < EquationsOfMotion 
    properties (GetAccess = public, SetAccess = private)
        Jacobian KinematicJacobian;
        JacobianRate KinematicJacobianRate; 
    end 
    methods (Access = public)
        function obj = KinematicEquations(equations,q,u,v)
            arguments
                equations (:,1) sym;
                q (:,1) sym;
                u (:,1) sym;
                v (:,1) sym = sym.empty(0,1);
            end
            qd = diff(q); 
            obj.Jacobian = KinematicJacobian(equations,qd);
            J = obj.Jacobian;
            obj.MassMatrix = eye(numel(qd),'sym');
            obj.ForcingVector = simplify(expand(J.u*u));
            obj.JacobianRate = KinematicJacobianRate(J,q,qd,obj.ForcingVector);
            x = [q;u;v];
            x = x(has(x,sym('t')));
            xd = diff(x);
            M = jacobian(obj.MassMatrix*qd,xd);
            f = -jacobian(obj.MassMatrix*qd - obj.ForcingVector,x);
            obj.Linearized = LinearizedEquations(M,simplify(expand(f)));
        end 
    end 
end