classdef Velocities
properties (GetAccess = public, SetAccess = private)
    States (:,1) sym;
    Rates (:,1) sym;
    Quasi (1,1) Coordinates;
    Auxiliary (1,1) Coordinates;
end
methods (Access = public)
function obj = Velocities(u,options)
    arguments
        u (:,1) sym;
        options.Auxiliary (:,1) sym = sym.empty();
    end
    obj.Quasi = Coordinates(u);
    obj.Auxiliary = Coordinates(options.Auxiliary);
end
end
end