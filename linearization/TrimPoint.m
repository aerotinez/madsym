classdef TrimPoint
    properties (GetAccess = public, SetAccess = private)
        x;
        x0; 
    end
    methods
        function obj = TrimPoint(q,u,v,F,q0,u0,v0,F0)
            arguments
                q (:,1) sym = sym.empty([0,1]);
                u (:,1) sym = sym.empty([0,1]);
                v (:,1) sym = sym.empty([0,1]);
                F (:,1) sym = sym.empty([0,1]); 
                q0 (:,1) sym = zeros(size(equations.q),'sym');
                u0 (:,1) sym = zeros(size(equations.u),'sym');
                v0 (:,1) sym = zeros(size(equations.v),'sym');
                F0 (:,1) sym = sym.empty([0,1]);
            end

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
        function disp(obj)
            disp(dictionary(obj.x,obj.x0));
        end
    end
end