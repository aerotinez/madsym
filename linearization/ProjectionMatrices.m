classdef ProjectionMatrices
    properties (GetAccess = public, SetAccess = private)
        C0;
        C1;
        C2;
    end
    properties (Access = private)
        hc;
        nhc;
        I;
        Jhc;
        Jnhcq;
        Jnhcu;
        Pqi;
        Pqd;
        Pui;
        Pud;
    end
    methods (Access = public)
        function obj = ProjectionMatrices(coordinates,kinematics,constraints)
            arguments
                coordinates (1,1) GeneralizedCoordinates;
                kinematics (1,1) KinematicEquations;
                constraints (1,1) Constraints; 
            end
            X = coordinates;
            K = kinematics;
            obj.hc = constraints.Holonomic;
            fk = K.ForcingVector;
            obj.nhc = simplify(expand(subs(constraints.Nonholonomic,X.qd,fk)));
            obj.I = eye(X.n);
            obj.Jhc = simplify(expand(jacobian(obj.hc,X.q)));
            obj.Jnhcq = simplify(expand(jacobian(obj.nhc,X.q)));
            obj.Jnhcu = simplify(expand(jacobian(obj.nhc,X.u)));
            P = PermutationMatrices(X);
            obj.Pqi = P.Pq_ind;
            obj.Pqd = P.Pq_dep;
            obj.Pui = P.Pu_ind;
            obj.Pud = P.Pu_dep;
            obj.C0 = simplify(expand(obj.computeC0()));
            obj.C1 = simplify(expand(obj.computeC1()));
            obj.C2 = simplify(expand(obj.computeC2()));
        end
    end
    methods (Access = private)
        function C0 = computeC0(obj)
            C0 = (obj.I - obj.Pqd*syminv(obj.Jhc*obj.Pqd)*obj.Jhc)*obj.Pqi;
        end
        function C1 = computeC1(obj)
            C1 = -obj.Pud*syminv(obj.Jnhcu*obj.Pud)*obj.Jnhcq;
        end
        function C2 = computeC2(obj)
            C2 = (obj.I - obj.Pud*syminv(obj.Jnhcu*obj.Pud)*obj.Jnhcu)*obj.Pui;
        end
    end
end