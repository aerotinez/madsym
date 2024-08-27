classdef ConstraintEquations < matlab.mixin.SetGet
    properties (GetAccess = public, SetAccess = protected)
        Configuration;
        Velocity;
        Acceleration;
    end
    methods (Access = public)
        function obj = ConstraintEquations(configuration,velocity)
            arguments
                configuration (:,1) sym = sym.empty(0,1);
                velocity (:,1) sym = sym.empty(0,1);
            end
            t = sym('t');
            obj.Configuration = configuration;
            obj.Velocity = [
                diff(configuration,t);
                velocity
                ];
            obj.Acceleration = diff(obj.Velocity,t);
        end
    end
end