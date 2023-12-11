classdef Coordinates
    properties (GetAccess = public, SetAccess = private)
        States;
        Rates;
        Independent;
        Dependent;
    end
    methods (Access = public)
        function obj = Coordinates(independent,dependent)
            arguments
                independent (:,1) DynamicVariable;
                dependent (:,1) DynamicVariable = DynamicVariable.empty(0,1);
            end
            obj.States = [independent.State,dependent.State].';
            obj.Rates = [independent.Rate,dependent.Rate].';
            f = @(states,rates)struct("States",states,"Rates",rates);
            obj.Independent = f([independent.State].',[independent.Rate].');
            obj.Dependent = f([dependent.State].',[dependent.Rate].');
        end
    end
end