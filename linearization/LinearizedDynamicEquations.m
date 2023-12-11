classdef LinearizedDynamicEquations < handle
properties (GetAccess = public, SetAccess = private)
    MassMatrix sym = sym.empty;
    ForcingMatrix sym = sym.empty;
end
methods (Access = public)
function obj = LinearizedDynamicEquations( ...
    generalized_coordinates, ...
    quasi_velocities, ...
    mass_matrix, ...
    forcing_vector ...
    )
    arguments
        generalized_coordinates (1,1) GeneralizedCoordinates = GeneralizedCoordinates();
        quasi_velocities (:,1) QuasiVariable = QuasiVariable.empty(0,1);
        mass_matrix sym = sym.empty;
        forcing_vector sym = sym.empty;
    end
    q = [generalized_coordinates.All.Position].';
    u = [quasi_velocities.Velocity].';
    ud = [quasi_velocities.Acceleration].';
    f0 = mass_matrix*ud;
    f1 = -forcing_vector;
    n = numel(q);
    k = numel(u);
    M = [zeros(k,n),mass_matrix];
    obj.MassMatrix = simplify(expand(M));
    f = -[jacobian(f0 + f1,q),jacobian(f1,u)];
    obj.ForcingMatrix = simplify(expand(f));
end
function setMassMatrix(obj, mass_matrix)
    obj.MassMatrix = mass_matrix;
end
function setForcingMatrix(obj, forcing_matrix)
    obj.ForcingMatrix = forcing_matrix;
end
end
end