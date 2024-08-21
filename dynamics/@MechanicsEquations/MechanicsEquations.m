classdef MechanicsEquations < MotionEquations
    properties (Access = protected)
        States;
        Kinematics;
        Dynamics;
    end
    methods (Access = public)
        function obj = MechanicsEquations(states,eomk,eomd)
            arguments
                states (1,1) StateVector;
                eomk (1,1) KinematicsEquations;
                eomd (1,1) DynamicsEquations;
            end
            x = [
                states.Coordinates.All;
                eomk.Inputs;
            ];

            M = blkdiag(eomk.MassMatrix,eomd.MassMatrix);

            f = [
                eomk.ForcingVector;
                eomd.ForcingVector;
            ];

            F = eomd.Inputs;

            obj@MotionEquations(x,M,f,F);
            obj.States = states;
            obj.Kinematics = eomk;
            obj.Dynamics = eomd;
        end
    end
end