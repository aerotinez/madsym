classdef GibbsAppell
    properties (GetAccess = public, SetAccess = private)
        q (:,1) sym = sym.empty(0,1);
        u (:,1) sym = sym.empty(0,1);
        v (:,1) sym = sym.empty(0,1);
        Coordinates GeneralizedCoordinates;
        Constraints Constraints;
        Inputs (:,1) sym = sym.empty(0,1);
    end
    methods (Access = public)
        function obj = GibbsAppell(coordinates,constraints,inputs)
            arguments
                coordinates (1,1) GeneralizedCoordinates;
                constraints (1,1) Constraints = Constraints();
                inputs (:,1) sym = sym.empty(0,1);
            end
            obj.Coordinates = coordinates;
            obj.q = obj.Coordinates.q;
            obj.u = obj.Coordinates.u_ind;
            obj.v = obj.Coordinates.v;
            obj.Constraints = constraints;
            obj.Inputs = inputs;
        end
        function eomk = kinematics(obj,equations)
            arguments
                obj (1,1) GibbsAppell;
                equations (:,1) sym;
            end 
            eq = [
                equations;
                obj.Constraints.Nonholonomic;
            ];
            u0 = [obj.u;zeros(obj.Coordinates.m,1)];
            eomk = KinematicEquations(eq,obj.q,u0,obj.v,obj.Inputs);
        end
        function eomd = dynamics(obj,kinematics,spatial_inertia,velocity_jacobian,wrench)
            arguments
                obj (1,1) GibbsAppell;
                kinematics (1,1) KinematicEquations;
                spatial_inertia (6,6) sym = sym.empty(6,6);
                velocity_jacobian (6,:) sym = sym.empty(6,0);
                wrench (6,1) sym = zeros(6,1,'sym');
            end
            K = kinematics;
            G = spatial_inertia;
            J = velocity_jacobian;
            F = wrench;
            
            W = K.Jacobian.u(:,1:numel(obj.u));
            Vbar = simplify(expand(J*W));
            Vdbar = jacobianRate(Vbar,obj.q,K.ForcingVector);
            
            V = Vbar*obj.u;
            wm = vec2skew(V(1:3));
            vm = vec2skew(V(4:6));
            ad = [
                wm,zeros(3);
                vm,wm
                ];
            
            M = simplify(expand(Vbar.'*G*Vbar));
            f = Vbar.'*F - Vbar.'*(G*Vdbar - ad.'*G*Vbar)*obj.u;

            x = [obj.q;obj.u;obj.v];
            xd = diff(x);
            Ml = jacobian(M*diff(obj.u),xd);
            Hl = -jacobian(M*diff(obj.u) - f,x);
            Gl = -jacobian(f,obj.Inputs);
            eoml = LinearizedEquations(x,Ml,Hl,Gl);

            eomd = DynamicEquations(obj.q,obj.u,obj.v,M,f,eoml);
        end
        function eoml = linearize(obj,eomk,eomd,eomv)
            arguments
                obj (1,1) GibbsAppell;
                eomk (1,1) KinematicEquations;
                eomd (1,1) DynamicEquations;
                eomv (1,1) AuxiliaryEquations;
            end
            
        end
    end
end