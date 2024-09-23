classdef LinearizedMotionEquations 
    properties (GetAccess = public, SetAccess = private)
        States;
        Inputs;
        MassMatrix;
        ForcingMatrix;
        InputMatrix;
    end
    methods (Access = public)
        function obj = LinearizedMotionEquations(x,M,H,G,u)
            arguments
                x (:,1) DynamicVariable;
                M sym;
                H sym;
                G sym = sym.empty(numel(x.All),0);
                u (:,1) DynamicVariable = DynamicVariable.empry(0,1);
            end
            obj.States = x;
            obj.MassMatrix = M;
            obj.ForcingMatrix = H;
            obj.Inputs = u;
            obj.InputMatrix = G;
        end 
    end 
end