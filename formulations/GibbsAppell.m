classdef GibbsAppell < handle
    properties (GetAccess = public, SetAccess = private)
        States StateVector;
        Inputs (:,1) sym = sym.empty(0,1);
        Constraints (1,1) Constraints;
        Equations MotionEquations;
        Trim TrimPoint;
        LinearizedEquations LinearizedMotionEquations; 
    end
    properties (Access = private)
        Kinematics MotionEquations;
        BodyDynamics MotionEquations;
        Dynamics MotionEquations;
        Auxiliary MotionEquations;
        LinearizationStates sym; 
    end
    methods (Access = public)
        function obj = GibbsAppell(states,constraints,inputs)
            arguments
                states (1,1) StateVector;
                constraints (1,1) Constraints = Constraints();
                inputs (:,1) sym = sym.empty(0,1);
            end
            obj.States = states;
            obj.LinearizationStates = [
                obj.States.Coordinates.All;
                obj.States.Speeds.Independent;
                obj.States.Auxiliary
            ];
            obj.Constraints = constraints;
            obj.validateConstraints();
            obj.Inputs = inputs;
        end
        function kinematics(obj,equations)
            arguments
                obj (1,1) GibbsAppell;
                equations (:,1) sym;
            end
            obj.validateKinematicEquations(equations);
            u = obj.States.Speeds.Independent;
            eomk = [
                u - equations;
                obj.Constraints.Nonholonomic
                ];
            q = obj.States.Coordinates.All;
            J = KinematicJacobian(eomk,diff(q));
            M = eye(obj.States.n,'sym');
            f = J.u(:,1:obj.States.k)*u;
            obj.Kinematics = MotionEquations(q,M,f,obj.Inputs);
        end
        function dynamics(obj,bodies)
            arguments
                obj (1,1) GibbsAppell;
                bodies (:,1) Body;
            end
            obj.BodyDynamics = arrayfun(@(b)obj.bodyDynamics(b),bodies);
            u = obj.States.Speeds.Independent;
            fB = @(f)arrayfun(f,obj.BodyDynamics,'uniform',0);
            sum3 = @(f)sum(cell2sym(reshape(fB(f),1,1,[])),3);
            M = sum3(@(b)b.MassMatrix);
            f = sum3(@(b)b.ForcingVector);
            obj.Dynamics = MotionEquations(u,M,f,obj.Inputs);
        end
        function auxiliary(obj,equations)
            arguments
                obj (1,1) GibbsAppell;
                equations (:,1) sym;
            end
            obj.validateAuxiliaryEquations(equations);
            qd = diff(obj.States.Coordinates.All);
            fk = obj.Kinematics.ForcingVector;
            eq = subs(equations,qd,fk);
            v = obj.States.Auxiliary;
            vd = diff(v);
            M = jacobian(eq,vd);
            obj.validateAuxiliaryMassMatrix(M);
            f = -subs(eq,vd,0.*vd);
            obj.Auxiliary = MotionEquations(v,M,f,obj.Inputs);
        end
        function equations(obj)
            Mk = obj.Kinematics.MassMatrix;
            Md = obj.Dynamics.MassMatrix;
            M = blkdiag(Mk,Md); 

            fk = obj.Kinematics.ForcingVector;
            fd = obj.Dynamics.ForcingVector;
            f = [fk;fd];
            if ~isempty(obj.Auxiliary)
                Mv = obj.Auxiliary.MassMatrix;
                M = blkdiag(M,Mv);
                fv = obj.Auxiliary.ForcingVector;
                f = [f;fv];
            end

            x = obj.LinearizationStates;
            obj.Equations = MotionEquations(x,M,f,obj.Inputs);
        end
        function trimPoint(obj,q0,u0,v0,F0)
            arguments
                obj (1,1) GibbsAppell;
                q0 (:,1) sym;
                u0 (:,1) sym;
                v0 (:,1) sym = sym.empty(0,1);
                F0 (:,1) sym = sym.empty(0,1);
            end
            q = obj.States.Coordinates.All;
            u = obj.States.Speeds.Independent;
            v = obj.States.Auxiliary;
            F = obj.Inputs;
            obj.Trim = TrimPoint(q,u,v,F,q0,u0,v0,F0);
        end 
        function linearize(obj)
            arguments
                obj (1,1) GibbsAppell;
            end
            x = obj.LinearizationStates;
            eomkl = obj.Kinematics.linearize(x);
            eomdl = obj.linearizeBodyDynamics();

            M = [
                eomkl.MassMatrix;
                eomdl.MassMatrix;
                ];

            H = [
                eomkl.ForcingMatrix;
                eomdl.ForcingMatrix;
                ];

            F = obj.Inputs;

            G = [
                eomkl.InputMatrix;
                eomdl.InputMatrix;
                ];
            
            if ~isempty(obj.Auxiliary)
                eomvl = obj.Auxiliary.linearize(x);
                M = [M;eomvl.MassMatrix];
                H = [H;eomvl.ForcingMatrix];
                G = [G;eomvl.InputMatrix];
            end

            obj.LinearizedEquations = LinearizedMotionEquations(x,M,H,F,G);
            obj.independentLinearizedEquations();
            obj.linearizedEquationsAtTrim();
        end
    end
    methods (Access = private) 
        function eomd = bodyDynamics(obj,body)
            arguments
                obj (1,1) GibbsAppell;
                body (1,1) Body;
            end
            u = obj.States.Speeds.Independent;
            fk = obj.Kinematics.ForcingVector;
            W = jacobian(fk,u);

            G = blkdiag(body.Inertia,body.Mass*eye(3,'sym'));
            J = body.Twist.jacobian(diff(obj.States.Coordinates.All));

            Vbar = J*W;
            Vdbar = jacobianRate(Vbar,obj.States.Coordinates.All,fk);

            V = Vbar*u;
            wm = vec2skew(V(1:3));
            vm = vec2skew(V(4:6));
            ad = [
                wm,zeros(3);
                vm,wm
                ];

            F = body.ActiveForces;

            M = Vbar.'*G*Vbar;
            f = Vbar.'*F - Vbar.'*(G*Vdbar - ad.'*G*Vbar)*u;
            eomd = MotionEquations(u,M,f,obj.Inputs); 
        end
        function fsub = subsTrim(obj,f)
            if isempty(obj.Trim)
                fsub = f;
                return
            end
            ovals = [diff(obj.Trim.x,sym('t'));obj.Trim.x];
            nvals = [diff(obj.Trim.x0,sym('t'));obj.Trim.x0];
            fsub = subs(f,ovals,nvals);
        end
        function eomdl = linearizeBodyDynamics(obj)
            x = obj.LinearizationStates;
            eomdl_bodies = arrayfun(@(b)b.linearize(x),obj.BodyDynamics);
            fB = @(f)arrayfun(f,eomdl_bodies,'uniform',0);
            sum3 = @(f)sum(cell2sym(reshape(fB(f),1,1,[])),3);
            M = sum3(@(b)b.MassMatrix);
            H = sum3(@(b)b.ForcingMatrix);
            F = obj.Inputs;
            G = sum3(@(b)b.InputMatrix);
            eomdl = LinearizedMotionEquations(x,M,H,F,G);
        end
        function independentLinearizedEquations(obj)
            P = PermutationMatrices(obj.States);
            C = dependentCoordinateProjection(obj.States,obj.Constraints);

            Hk = obj.LinearizedEquations.ForcingMatrix(:,1:obj.States.n);
            Hdv = obj.LinearizedEquations.ForcingMatrix(:,obj.States.n + 1:end);

            x = obj.LinearizationStates;
            P = blkdiag(P.q_ind,eye(obj.States.k + obj.States.p,'sym'));
            M = obj.LinearizedEquations.MassMatrix;
            H = [Hk*C,Hdv];
            F = obj.Inputs;
            G = obj.LinearizedEquations.InputMatrix;

            obj.LinearizedEquations = LinearizedMotionEquations(x,M,H,F,G,P);
        end
        function linearizedEquationsAtTrim(obj)
            if isempty(obj.Trim)
                return
            end
            x = obj.LinearizationStates;
            M = obj.subsTrim(obj.LinearizedEquations.MassMatrix);
            H = obj.subsTrim(obj.LinearizedEquations.ForcingMatrix);
            F = obj.Inputs;
            G = obj.subsTrim(obj.LinearizedEquations.InputMatrix);
            P = obj.LinearizedEquations.PermutationMatrix;
            obj.LinearizedEquations = LinearizedMotionEquations(x,M,H,F,G,P);
        end
    end
    methods (Access = private)
        function validateConstraints(obj)
            if ~isequal(size(obj.Constraints.Holonomic,1),obj.States.l)
                msga = "Number of holonomic constraints must match number of ";
                msgb = "dependent coordinates.";
                error(msga + msgb);
            end
            if ~isequal(size(obj.Constraints.Nonholonomic,1),obj.States.m)
                msga = "Number of nonholonomic constraints must match number ";
                msgb = "of dependent speeds.";
                error(msga + msgb);
            end
        end
        function validateKinematicEquations(obj,equations)
            if ~isequal(size(equations,1),obj.States.k)
                msga = "Number of kinematic equations must match number of ";
                msgb = "independent speeds.";
                error(msga + msgb);
            end
        end
        function validateAuxiliaryEquations(obj,equations)
            if ~isequal(size(equations,1),obj.States.p)
                msga = "Number of auxiliary equations must match number of ";
                msgb = "auxiliary coordinates.";
                error(msga + msgb);
            end
        end
        function validateAuxiliaryMassMatrix(obj,mass_matrix)
            if has(mass_matrix,obj.States.Speeds.Independent)
                msga = "Auxiliary mass matrix cannot depend on independent ";
                msgb = "speeds.";
                error(msga + msgb);
            end
        end
    end
end