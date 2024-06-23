classdef DynamicMotionEquations < MotionEquations 
    properties (Access = private)
        f0;
        f1;
        f2;
    end
    methods (Access = public)
        function obj = DynamicMotionEquations(x,M,f0,f1,f2,F)
            arguments
                x (:,1) sym {mustBeNonempty};
                M sym {mustBeNonempty};
                f0 (:,1) sym {mustBeNonempty};
                f1 (:,1) sym {mustBeNonempty};
                f2 (:,1) sym {mustBeNonempty};
                F sym = sym.empty(0,1);
            end
            obj@MotionEquations(x,M,f0 + f1 + f2,F);
            obj.f0 = f0;
            obj.f1 = f1;
            obj.f2 = f2;
        end
    end
end