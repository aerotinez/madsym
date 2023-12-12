classdef KinematicEquations < MechanicalEquations 
    properties (GetAccess = public, SetAccess = private)
        Jacobian KinematicJacobian;
        JacobianRate KinematicJacobianRate; 
    end 
    methods (Access = public)
        function obj = KinematicEquations(equations,q,u,v,F)
            arguments
                equations (:,1) sym;
                q (:,1) sym;
                u (:,1) sym;
                v (:,1) sym = sym.empty(0,1);
                F (:,1) sym = sym.empty(0,1);
            end
            obj.q = q;
            obj.qd = diff(obj.q);
            obj.u = u;
            obj.ud = diff(obj.u);
            obj.v = v;
            obj.vd = diff(obj.v); 
            obj.Jacobian = KinematicJacobian(equations,obj.qd);
            J = obj.Jacobian;
            obj.MassMatrix = eye(numel(obj.qd),'sym');
            obj.ForcingVector = simplify(expand(J.u*u));
            fk = obj.ForcingVector;
            obj.JacobianRate = KinematicJacobianRate(J,obj.q,obj.qd,fk);
            x = [q;u;v];
            x = x(has(x,sym('t')));
            xd = diff(x);
            M = jacobian(obj.MassMatrix*obj.qd,xd);
            H = -jacobian(obj.MassMatrix*obj.qd - fk,x);
            H = simplify(expand(H));
            G = simplify(expand(-jacobian(-fk,F)));
            obj.Linearized = LinearizedEquations(x,M,H,G);
        end 
    end 
end