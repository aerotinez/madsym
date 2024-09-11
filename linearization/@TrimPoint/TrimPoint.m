classdef TrimPoint
    properties (GetAccess = public, SetAccess = private)
        x;
        x0;
        q;
        u;
        v;
        F;
        q0;
        u0;
        v0;
        F0;
    end
    methods
        function obj = TrimPoint(states,inputs,q0,u0,v0,F0)
            arguments
                states (1,1) StateVector; 
                inputs (:,1) sym = sym.empty([0,1]);
                q0 (:,1) sym = sym.empty([0,1]);
                u0 (:,1) sym = sym.empty([0,1]);
                v0 (:,1) sym = sym.empty([0,1]);
                F0 (:,1) sym = sym.empty([0,1]);
            end
            
            obj.q = states.Coordinates.All;
            obj.u = states.Speeds.All;
            obj.v = sym.empty([0,1]);
            if ~isempty(states.Auxiliary)
                obj.v = states.Auxiliary.All;
            end
            obj.F = inputs;

            obj.q0 = q0;
            obj.u0 = u0;
            obj.v0 = v0;
            obj.F0 = F0;

            obj.x = [
                obj.q;
                obj.u;
                obj.v;
                obj.F;
                ];

            obj.x0 = [
                q0;
                u0;
                v0;
                F0;
                ];
        end 
    end
end