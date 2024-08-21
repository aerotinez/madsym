classdef DynamicsMethod < handle
    properties (GetAccess = public, SetAccess = protected)
        States;
        Inputs;
        Kinematics;
        Constraints = ConstraintEquations.empty(0,1);
        Bodies;
        BodyDynamics;
        Dynamics;
    end
    methods (Access = public)
        function obj = DynamicsMethod(states,inputs)
            arguments
                states (1,1) StateVector;
                inputs (:,1) sym = sym.empty(0,1);
            end
            obj.States = states;
            obj.Inputs = inputs;
        end
    end
    methods (Access = public, Abstract)
        kinematics(obj,kdes);
        dynamics(obj,bodies);
    end
    methods (Access = protected, Abstract)
        eomd = bodyDynamics(obj,body);
    end
end