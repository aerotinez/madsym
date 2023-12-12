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
        function eoml = linearize(obj,eom,trim_point)
            arguments
                obj (1,1) GibbsAppell;
                eom (1,1) EquationsOfMotion;
                trim_point (1,1) TrimPoint;
            end
            P = PermutationMatrices(obj.Coordinates);
            C = dependentCoordinateProjection(obj.Coordinates,obj.Constraints);

            ovals = [diff(trim_point.x,sym('t'));trim_point.x];
            nvals = [diff(trim_point.x0,sym('t'));trim_point.x0];
            f = @(eq)simplify(expand(subs(eq,ovals,nvals)));

            M = f(eom.Linearized.MassMatrix);

            Hq = eom.Linearized.ForcingMatrix(:,1:numel(obj.q))*C;
            Huv = eom.Linearized.ForcingMatrix(:,numel(obj.q)+1:end);
            H = f([Hq,Huv]);

            G = f(eom.Linearized.InputMatrix);

            eoml = LinearizedEquations([obj.q;obj.u;obj.v],M,H,G);
        end
    end
end