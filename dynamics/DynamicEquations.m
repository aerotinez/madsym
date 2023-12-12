classdef DynamicEquations < MechanicalEquations
    methods (Access = public)
        function obj = DynamicEquations(q,u,v,mass_matrix,forcing_vector,linearized)
            arguments
                q (:,1) sym;
                u (:,1) sym;
                v (:,1) sym;
                mass_matrix sym;
                forcing_vector sym;
                linearized (1,1) LinearizedEquations;
            end
            obj.q = q;
            obj.u = u;
            obj.v = v;
            obj.qd = diff(q);
            obj.ud = diff(u);
            obj.vd = diff(v);
            obj.MassMatrix = mass_matrix;
            obj.ForcingVector = forcing_vector;
            obj.Linearized = linearized;
        end
    end
end