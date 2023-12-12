classdef AuxiliaryEquations < MechanicalEquations
    methods (Access = public)
        function obj = AuxiliaryEquations(kinematics,equations,inputs)
            arguments
                kinematics KinematicEquations;
                equations (:,1) sym; 
                inputs (:,1) sym = sym.empty(0,1);
            end
            obj.q = kinematics.q;
            obj.u = kinematics.u;
            obj.v = kinematics.v;
            obj.qd = kinematics.qd;
            obj.ud = kinematics.ud;
            obj.vd = kinematics.vd;
            obj.MassMatrix = jacobian(equations,obj.vd);
            fk = kinematics.ForcingVector;
            fv = -subs(equations,obj.vd,0.*obj.vd);
            obj.ForcingVector = subs(fv,obj.qd,fk);
            x = [obj.q;obj.u;obj.v];
            x = x(has(x,sym('t')));
            xd = diff(x);
            M = simplify(expand(jacobian(obj.MassMatrix*obj.vd,xd)));
            H = -jacobian(obj.MassMatrix*obj.vd + obj.ForcingVector,x);
            G = -jacobian(obj.MassMatrix*obj.vd + obj.ForcingVector,inputs);
            obj.Linearized = LinearizedEquations(x,M,H,G);
        end
    end
end