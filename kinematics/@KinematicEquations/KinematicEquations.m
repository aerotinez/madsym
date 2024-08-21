classdef KinematicEquations < MotionEquations
    properties (GetAccess = public, SetAccess = protected)
        Jacobian;
    end
    methods (Access = public)
        function obj = KinematicEquations(q,eq,u)
            arguments
                q (1,1) GeneralizedCoordinates;
                eq (:,1) sym {mustBeNonempty};
                u (1,1) GeneralizedCoordinates;
            end
            [M,f] = massMatrixForm(eq,q.All);
            uf = setdiff(u.All, findSymType(f,"symfun"));
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
            obj@MotionEquations(q.All,M,f,F);
            obj.Jacobian = jacobian(obj.ForcingVector,obj.Inputs);
        end
    end 
end