classdef ConstraintEquations 
    properties (GetAccess = public, SetAccess = protected)
        Configuration;
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
            t = sym('t');
            qd = diff(q.All,t);
            obj.Configuration = configuration;

            nhc = [
                diff(configuration,t);
                velocity
                ];

            obj.Jacobian = massMatrixForm(nhc,q.All);
            fAd = @(a)jacobian(a,q.All)*qd;
            a = reshape(obj.Jacobian,[],1);
            obj.JacobianRate = reshape(arrayfun(fAd,a),size(obj.Jacobian));
        end
    end
end