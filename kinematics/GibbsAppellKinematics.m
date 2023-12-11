classdef GibbsAppellKinematics < KinematicsStrategy
    properties (Access = private)
        Coordinates;
        Equations;
        Constraints;
    end
    methods (Access = public)
        function obj = GibbsAppellKinematics(coordinates,equations,constraints)
            arguments
                coordinates (1,1) GeneralizedCoordinates;
                equations (:,1) sym;
                constraints (:,1) Constraints = Constraints();
            end
            obj.Coordinates = coordinates;
            obj.Equations = equations;
            obj.Constraints = constraints;
        end
        function eomk = kinematicEquations(obj)
            q = obj.Coordinates.q;
            u = [obj.Coordinates.u_ind;zeros(obj.Coordinates.m,1)];
            v = obj.Coordinates.v;
            eq = [
                obj.Equations;
                obj.Constraints.Nonholonomic;
            ];
            eomk = KinematicEquations(eq,q,u,v);
        end
    end
end