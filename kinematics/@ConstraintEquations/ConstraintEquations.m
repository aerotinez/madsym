classdef ConstraintEquations 
    properties (GetAccess = public, SetAccess = protected)
        Configuration;
        Velocity;
        Acceleration;
        Jacobian;
        JacobianRate;
    end
    methods (Access = public)
        function obj = ConstraintEquations(q,configuration,velocity)
            arguments
                q (1,1) GeneralizedCoordinates;
                configuration (:,1) sym = sym.empty(0,1);
                velocity (:,1) sym = sym.empty(0,1);
            end
            % Generalized coordinates
            t = sym('t');
            qd = diff(q.All,t);

            % Holonomic constraints
            obj.Configuration = configuration;

            % Nonholonomic constraints
            nhc = [
                diff(configuration,t);
                velocity
                ];

            [A,f] = massMatrixForm(nhc,q.All);
            obj.Jacobian = A;
            obj.Velocity = MotionEquations(q.All,A,f);

            % Acceleration constraints
            fAd = @(a)jacobian(a,q.All)*qd;
            Ad = reshape(arrayfun(fAd,reshape(A,[],1)),size(A));
            obj.JacobianRate = Ad;
            f = -Ad*qd;
            obj.Acceleration = MotionEquations(qd,A,f);
        end
    end
end