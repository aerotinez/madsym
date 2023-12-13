classdef PermutationMatrices
    properties (GetAccess = public, SetAccess = private)
        q_ind;
        q_dep;
        u_ind;
        u_dep;
    end
    methods (Access = public)
        function obj = PermutationMatrices(states)
            arguments
                states (1,1) StateVector;
            end
            q = states.Coordinates.All;
            qind = states.Coordinates.Independent;
            qdep = states.Coordinates.Dependent;

            u = states.Speeds.All;
            uind = states.Speeds.Independent;
            udep = states.Speeds.Dependent;

            n = states.n;
            l = states.l;
            m = states.m;
            k = states.k;

            f = @(q,x)double(has(q,x).');
            fPq = @(x)cell2mat(arrayfun(@(x)f(q,x),x,'uniform',0));
            Pq = fPq([qind;qdep])\eye(n);

            obj.q_ind = Pq(:,1:n-l);
            obj.q_dep = Pq(:,(n-l+1):end);

            fPu = @(x)cell2mat(arrayfun(@(x)f(u,x),x,'uniform',0));
            Pu = fPu([uind;udep])\eye(n);

            obj.u_ind = Pu*[
                eye(k); 
                zeros(m,k)
                ];

            obj.u_dep = Pu*[
                zeros(k,m); 
                eye(m)
                ];
        end
    end
end