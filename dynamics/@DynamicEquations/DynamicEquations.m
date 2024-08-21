classdef DynamicEquations < MotionEquations
    properties (GetAccess = public, SetAccess = protected, Hidden = true)
        f0;
        f1;
        f2;
    end
    methods (Access = public)
        function obj = DynamicEquations(u,M,f0,f1,f2,F)
            arguments
                u (1,1) GeneralizedCoordinates;
                M sym;
                f0 (:,1) sym;
                f1 (:,1) sym;
                f2 (:,1) sym;
                F (:,1) sym = sym.empty(0,1);
            end
            obj@MotionEquations(u.All,M,f0 + f1 + f2,F);
            obj.f0 = f0;
            obj.f1 = f1;
            obj.f2 = f2;
        end
    end
end
