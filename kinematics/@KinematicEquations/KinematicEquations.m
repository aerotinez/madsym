classdef KinematicEquations < MotionEquations
    properties (GetAccess = public, SetAccess = private)
        Jacobian;
        JacobianRate;
    end
    methods (Access = public)
        function obj = KinematicEquations(q,kdes,u)
            arguments
                q (:,1) DynamicVariable;
                kdes (:,1) sym {mustBeNonempty};
                u (:,1) DynamicVariable;
            end
            [M,f] = massMatrixForm(kdes,q.state());
            uf = setdiff(u.state,findSymType(f,"symfun"));
            if isempty(uf)
                F = u;
            elseif isempty(setdiff(uf,u.dependent))
                F = u.independent();
            else
                msga = "Kinematic equations must be in terms of generalized ";
                msgb = "coordinate rates and at least the independent ";
                msgc = "generalized speeds.";
                error(msga + msgb + msgc); 
            end
            obj@MotionEquations(q,eye(size(M)),simplify(expand(syminv(M)))*f,F);
            obj.Jacobian = jacobian(obj.ForcingVector,F.state);
            fJd = @(j)jacobian(j,q.state())*obj.ForcingVector;
            J = obj.Jacobian;
            obj.JacobianRate = reshape(arrayfun(fJd,reshape(J,[],1)),size(J));
        end
    end
end