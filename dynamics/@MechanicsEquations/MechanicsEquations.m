classdef MechanicsEquations < MotionEquations
    properties (GetAccess = public, SetAccess = protected, Hidden = true)
        StateVector;
        Constraints;
        Kinematics;
        BodyDynamics;
    end
    methods (Access = public)
        function obj = MechanicsEquations(states,eomk,eomd_list,eomc)
            arguments
                states (1,1) StateVector;
                eomk (1,1) KinematicEquations;
                eomd_list (:,1) DynamicEquations;
                eomc (:,1) ConstraintEquations = ConstraintEquations.empty(0,1);
            end

            x = [
                states.Coordinates.All;
                eomk.Inputs
                ];
            
            eomd = sum(eomd_list);
            F = eomd.Inputs;

            M = blkdiag(eomk.MassMatrix,eomd.MassMatrix);

            f = [
                eomk.ForcingVector;
                eomd.ForcingVector
                ];

            if ~isempty(states.Speeds.Dependent)
                A = eomc.Jacobian;

                M = [
                    M;
                    zeros(size(A,1),size(M,2) - size(A,2)),A
                    ];

                fa = eomc.Acceleration.ForcingVector;

                f = [
                    f;
                    fa
                    ];
            end

            obj@MotionEquations(x,M,f,F);
            obj.StateVector = states;
            obj.Constraints = eomc;
            obj.Kinematics = eomk;
            obj.BodyDynamics = eomd_list;
        end
    end
end