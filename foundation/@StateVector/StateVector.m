classdef StateVector
    properties (GetAccess = public, SetAccess = private)
        Coordinates;
        Speeds;
        Auxiliary;
    end
    properties (GetAccess = public, SetAccess = private, Hidden = true)
        n; % number of coordinates
        l; % number of dependent coordinates
        k; % number of independent speeds
        m; % number of dependent speeds
        p; % number of auxiliary variables
        P; % pertubation matrix
    end
    methods (Access = public)
        function obj = StateVector(q,u,v)
            arguments
                q (1,1) GeneralizedCoordinates;
                u (1,1) GeneralizedCoordinates;
                v (1,1) GeneralizedCoordinates = GeneralizedCoordinates();
            end
            obj.Coordinates = q;
            obj.Speeds = u;
            obj.Auxiliary = v;
            obj.n = numel(obj.Coordinates.All);
            obj.l = numel(obj.Coordinates.Dependent);
            obj.k = numel(obj.Speeds.Independent);
            obj.m = numel(obj.Speeds.Dependent);
            obj.p = numel(obj.Auxiliary.All);
            Pq = obj.Coordinates.Pind;
            Pu = obj.Speeds.Pind;
            Pv = obj.Auxiliary.Pind;
            obj.P = blkdiag(Pq,Pu,Pv);
        end
    end 
end