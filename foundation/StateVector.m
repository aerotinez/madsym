classdef StateVector
    properties (GetAccess = public, SetAccess = private)
        Coordinates GeneralizedCoordinates;
        Speeds GeneralizedCoordinates;
        Auxiliary (:,1) sym = sym.empty(0,1);
    end
    properties (GetAccess = public, SetAccess = private, Hidden = true)
        n; % number of coordinates
        l; % number of dependent coordinates
        k; % number of independent speeds
        m; % number of dependent speeds
        p; % number of auxiliary variables
    end
    methods (Access = public)
        function obj = StateVector(coordinates,speeds,auxiliary)
            arguments
                coordinates (1,1) GeneralizedCoordinates;
                speeds (1,1) GeneralizedCoordinates;
                auxiliary (:,1) sym = sym.empty(0,1);
            end
            obj.Coordinates = coordinates;
            obj.Speeds = speeds;
            obj.Auxiliary = auxiliary;
            obj.validateAuxiliarySpeeds();
            obj.n = numel(obj.Coordinates.All);
            obj.l = numel(obj.Coordinates.Dependent);
            obj.k = numel(obj.Speeds.Independent);
            obj.m = numel(obj.Speeds.Dependent);
            obj.p = numel(obj.Auxiliary);
        end
    end
    methods (Access = private)
        function validateAuxiliarySpeeds(obj)
            if isempty(obj.Auxiliary)
                return
            end
            if ~all(isDynamicVariable(obj.Auxiliary))
                error('Auxiliary variables must be dynamic variables');
            end
        end
    end
end