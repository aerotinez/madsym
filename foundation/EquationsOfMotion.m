classdef EquationsOfMotion < MechanicalEquations
    methods (Access = public)
        function obj = EquationsOfMotion(eomk,eomd,eomv)
            arguments
                eomk (1,1) KinematicEquations;
                eomd (1,1) DynamicEquations;
                eomv (:,1) AuxiliaryEquations = AuxiliaryEquations.empty(0,1);
            end
            obj.q = eomk.q;
            obj.qd = eomk.qd;
            obj.u = eomd.u;
            obj.ud = eomd.ud;
            obj.v = eomv.v;
            obj.vd = eomv.vd;

            M = blkdiag(eomk.MassMatrix,eomd.MassMatrix,eomv.MassMatrix);
            obj.MassMatrix = M;

            f = [
                eomk.ForcingVector;
                eomd.ForcingVector;
                eomv.ForcingVector
                ];
            obj.ForcingVector = f;

            x = [obj.q;obj.u;obj.v];

            Ml = [
                eomk.Linearized.MassMatrix;
                eomd.Linearized.MassMatrix;
                eomv.Linearized.MassMatrix
            ];

            Hl = [
                eomk.Linearized.ForcingMatrix;
                eomd.Linearized.ForcingMatrix;
                eomv.Linearized.ForcingMatrix
            ];

            Gl = [
                eomk.Linearized.InputMatrix;
                eomd.Linearized.InputMatrix;
                eomv.Linearized.InputMatrix
            ];

            obj.Linearized = LinearizedEquations(x,Ml,Hl,Gl);
        end
    end
end