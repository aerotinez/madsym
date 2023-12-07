classdef GeneralizedCoordinates
properties (GetAccess = public, SetAccess = private)
    States (:,1) sym;
    Rates (:,1) sym;
    Independent (1,1) Coordinates;
    Dependent (1,1) Coordinates;
end
methods (Access = public)
function obj = GeneralizedCoordinates(q,options)
    arguments
        q (:,1) sym;
        options.Dependent (:,1) sym = sym.empty;
    end
    obj.Independent = Coordinates(q);
    obj.Dependent = Coordinates(options.Dependent);
    obj.States = [
        obj.Independent.States;
        obj.Dependent.States
        ];
    obj.Rates = [
        obj.Independent.Rates;
        obj.Dependent.Rates
        ];
end
end
end