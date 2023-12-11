classdef AuxiliaryEquations < EquationsOfMotion
    methods (Access = public)
        function obj = AuxiliaryEquations(v,equations,q,u,kinematics)
            arguments
                v (:,1) sym;
                equations (:,1) sym;
                q (:,1) sym;
                u (:,1) sym;
                kinematics KinematicEquations;
            end
            vd = diff(v);
            obj.MassMatrix = jacobian(equations,vd);
            fk = kinematics.ForcingVector;
            fv = -subs(equations,vd,0.*vd);
            qd = diff(q);
            obj.ForcingVector = subs(fv,qd,fk);
            x = [q;u;v];
            x = x(has(x,sym('t')));
            xd = diff(x);
            M = simplify(expand(jacobian(obj.MassMatrix*vd,xd)));
            f = -jacobian(obj.MassMatrix*vd + obj.ForcingVector,x);
            obj.Linearized = LinearizedEquations(M,f);
        end
    end
end