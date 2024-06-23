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
        function obj = TrimPoint(q,u,v,F,q0,u0,v0,F0)
            arguments
                q (:,1) sym = sym.empty([0,1]);
                u (:,1) sym = sym.empty([0,1]);
                v (:,1) sym = sym.empty([0,1]);
                F (:,1) sym = sym.empty([0,1]); 
                q0 (:,1) sym = sym.empty([0,1]);
                u0 (:,1) sym = sym.empty([0,1]);
                v0 (:,1) sym = sym.empty([0,1]);
                F0 (:,1) sym = sym.empty([0,1]);
            end
            obj.q = q;
            obj.u = u;
            obj.v = v;
            obj.F = F;
            obj.q0 = q0;
            obj.u0 = u0;
            obj.v0 = v0;
            obj.F0 = F0;

            obj.x = [
                q;
                u;
                v;
                F;
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