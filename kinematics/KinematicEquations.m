classdef KinematicEquations < EquationsOfMotion 
    properties (GetAccess = public, SetAccess = private)
        Jqd sym;
        Ju sym;
        Jdqd sym;
        Jdu sym;
    end
    methods (Access = public)
        function obj = KinematicEquations(coordinates,equations)
            arguments
                coordinates (1,1) GeneralizedCoordinates;
                equations (:,1) sym;
            end
            obj.Jqd = jacobian(equations,coordinates.qd);
            obj.Ju = simplify(expand(syminv(Jqd)));
            obj.MassMatrix = eye(coordinates.n,'sym');
            obj.ForcingVector = obj.Ju*coordinates.u;
            obj.Jdqd = jacobianRate(obj.Jqd,coordinates.q,coordinates.qd);
            obj.Jdu = jacobianRate(obj.Ju,coordinates.q,obj.ForcingVector);
        end
    end
end