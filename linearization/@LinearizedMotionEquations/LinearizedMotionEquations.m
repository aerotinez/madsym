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
                x (1,1) GeneralizedCoordinates;
                M sym;
                H sym;
                G sym = sym.empty(numel(x.All),0);
                u (1,1) GeneralizedCoordinates = GeneralizedCoordinates();
            end
            obj.States = x;
            obj.MassMatrix = M;
            obj.ForcingMatrix = H;
            obj.Inputs = u;
            obj.InputMatrix = G;
        end 
    end 
end