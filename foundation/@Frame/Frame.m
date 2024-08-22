classdef Frame
    properties (GetAccess = public, SetAccess = private)
        x;
        y;
        z;
    end
    methods (Access = public)
        function obj = Frame(R)
            arguments
                R (3,3) sym = eye(3,'sym');
            end 
            obj.x = R(:,1);
            obj.y = R(:,2);
            obj.z = R(:,3);
        end 
    end
end