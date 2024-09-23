classdef ConstraintEquations 
    properties (GetAccess = public, SetAccess = protected)
        Configuration;
        Jacobian;
        JacobianRate;
    end
    methods (Access = public)
        function obj = ConstraintEquations(q,configuration,velocity)
            arguments
                q (:,1) DynamicVariable;
                configuration (:,1) sym = sym.empty(0,1);
                velocity (:,1) sym = sym.empty(0,1);
            end
            obj.Configuration = configuration;

            nhc = [
                diff(configuration,sym('t'));
                velocity
                ];

            obj.Jacobian = massMatrixForm(nhc,q.state());
            fAd = @(a)jacobian(a,q.state())*q.rate();
            a = reshape(obj.Jacobian,[],1);
            obj.JacobianRate = reshape(arrayfun(fAd,a),size(obj.Jacobian));
        end
    end
end