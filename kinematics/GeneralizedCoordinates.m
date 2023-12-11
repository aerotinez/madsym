classdef GeneralizedCoordinates
    properties (GetAccess = public, SetAccess = private)
        q;
        qd;
        qdd;
        q_ind;
        qd_ind;
        qdd_ind;
        q_dep;
        qd_dep;
        qdd_dep;
        u;
        ud;
        u_ind;
        ud_ind;
        u_dep;
        ud_dep;
        v;
        vd;
        n;
        l;
        m;
        k;
        p;
    end
    methods (Access = public)
        function obj = GeneralizedCoordinates(joint,quasi,auxiliary)
            arguments
                joint (1,1) Coordinates;
                quasi (:,1) Coordinates {mustBeScalarOrEmpty} = Coordinates.empty(0,1);
                auxiliary (:,1) Coordinates {mustBeScalarOrEmpty} = Coordinates.empty(1,0);
            end
            obj.q = joint.States;
            obj.qd = joint.Rates;
            obj.qdd = diff(joint.Rates);
            obj.q_ind = joint.Independent.States;
            obj.qd_ind = joint.Independent.Rates;
            obj.qdd_ind = diff(joint.Independent.Rates);
            obj.q_dep = joint.Dependent.States;
            obj.qd_dep = joint.Dependent.Rates;
            obj.qdd_dep = diff(joint.Dependent.Rates);
            obj.u = quasi.States;
            obj.ud = quasi.Rates;
            obj.u_ind = quasi.Independent.States;
            obj.ud_ind = quasi.Independent.Rates;
            obj.u_dep = quasi.Dependent.States;
            obj.ud_dep = quasi.Dependent.Rates;
            obj.v = auxiliary.States;
            obj.vd = auxiliary.Rates;
            obj.n = numel(obj.q);
            obj.l = numel(obj.q_dep);
            obj.m = numel(obj.u_dep);
            obj.k = obj.n - obj.m;
            obj.p = numel(obj.v);
        end
    end
end