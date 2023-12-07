classdef GeneralizedCoordinates
properties (GetAccess = public, SetAccess = private)
    States (:,1) sym = sym.empty([0,1]);
    Rates (:,1) sym = sym.empty([0,1]);
    Independent Coordinates;
    Dependent Coordinates;
end
methods (Access = public)
function obj = GeneralizedCoordinates(q,options)
    arguments
        q (:,1) sym = sym.empty([0,1]);
        options.Dependent (:,1) sym = sym.empty([0,1]);
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