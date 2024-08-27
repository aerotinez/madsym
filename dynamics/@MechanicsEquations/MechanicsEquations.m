classdef MechanicsEquations < MotionEquations
    properties (Access = protected)
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

            M = blkdiag(eomk.MassMatrix,eomd.MassMatrix);

            f = [
                eomk.ForcingVector;
                eomd.ForcingVector
                ];

            F = eomd.Inputs;

            obj@MotionEquations(x,M,f,F);
            obj.StateVector = states;
            obj.Constraints = eomc;
            obj.Kinematics = eomk;
            obj.BodyDynamics = eomd_list;
        end
    end
end