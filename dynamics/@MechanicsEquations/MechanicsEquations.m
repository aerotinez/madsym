classdef MechanicsEquations < MotionEquations
    properties (GetAccess = public, SetAccess = protected, Hidden = true)
        Constraints;
        Kinematics;
        BodyDynamics;
        Auxiliary;
    end
    methods (Access = public)
        function obj = MechanicsEquations(eomk,eomd_list,eomc,eomv)
            arguments
                eomk (1,1) KinematicEquations;
                eomd_list (:,1) DynamicEquations;
                eomc (:,1) ConstraintEquations = ConstraintEquations.empty(0,1);
                eomv (:,1) MotionEquations = MotionEquations.empty(0,1); 
            end

            x = [
                eomk.States;
                eomd_list(1).States;
                ];

            eqns = [
                sym(eomk);
                sym(sum(eomd_list))
                ];

%             l = numel(eomc.configuration);
%             m = numel(eomc.velocity);
%             if ~isempty(eomc) && m > l
%                 A = eomc.Jacobian;
%                 Ad = eomc.JacobianRate;
% 
%                 eqns = [
%                     eqns;
%                     A*eomk.Inputs.rate() + Ad*eomk.Inputs.state()
%                     ];
%             end

            if ~isempty(eomv)
                x = [
                    x;
                    eomv.States
                    ];

                eqns = [
                    eqns;
                    sym(eomv)
                    ];
            end

            [M,f] = massMatrixForm(eqns,x.state);
            obj@MotionEquations(x,M,f,eomd_list(1).Inputs);
            obj.Constraints = eomc;
            obj.Kinematics = eomk;
            obj.BodyDynamics = eomd_list;
            obj.Auxiliary = eomv;
        end
    end
end