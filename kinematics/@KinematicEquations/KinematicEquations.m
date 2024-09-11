classdef KinematicEquations < MotionEquations
    properties (GetAccess = public, SetAccess = private)
        Jacobian;
        JacobianRate;
    end
    methods (Access = public)
        function obj = KinematicEquations(q,eq,u)
            arguments
                q (1,1) GeneralizedCoordinates;
                eq (:,1) sym {mustBeNonempty};
                u (1,1) GeneralizedCoordinates;
            end
            [M,f] = massMatrixForm(eq,q.All);
            M = simplify(expand(M));
            uf = setdiff(u.All,findSymType(f,"symfun"));
            if isempty(uf)
                F = u.All;
            elseif isempty(setdiff(uf,u.Dependent))
                F = u.Independent;
            else
                msga = "Kinematic equations must be in terms of generalized ";
                msgb = "coordinate rates and at least the independent ";
                msgc = "generalized speeds.";
                error(msga + msgb + msgc); 
            end
            f = syminv(M)*f;
            M = eye(size(M));
            obj@MotionEquations(q.All,M,f,F);
            obj.Jacobian = jacobian(obj.ForcingVector,obj.Inputs);
            fJd = @(j)jacobian(j,q.All)*obj.ForcingVector;
            J = obj.Jacobian;
            obj.JacobianRate = reshape(arrayfun(fJd,reshape(J,[],1)),size(J));
        end
    end
end