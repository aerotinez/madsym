classdef Point
    properties (GetAccess = public, SetAccess = private)
        x;
        y; 
        z;
    end
    methods
        function obj = Point(p)
            arguments
                p (3,1) sym = zeros(3,1,'sym');
            end
            p = simplify(expand(p));
            obj.x = p(1);
            obj.y = p(2);
            obj.z = p(3);
        end 
    end
end