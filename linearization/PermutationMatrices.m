classdef PermutationMatrices
    properties (GetAccess = public, SetAccess = private)
        Pq_ind;
        Pq_dep;
        Pu_ind;
        Pu_dep;
    end
    methods (Access = public)
        function obj = PermutationMatrices(coordinates)
            arguments
                coordinates (1,1) GeneralizedCoordinates;
            end
            X = coordinates;
            f = @(q,x)double(has(q,x).');
            fPq = @(x)cell2mat(arrayfun(@(x)f(X.q,x),x,'uniform',0));
            Pq = fPq([X.q_ind;X.q_dep])\eye(X.n);

            obj.Pq_ind = Pq*[
                eye(X.n - X.l); 
                zeros(X.l,X.n - X.l)
                ];

            obj.Pq_dep = Pq*[
                zeros(X.n - X.l,X.l); 
                eye(X.l)
                ];

            fPu = @(x)cell2mat(arrayfun(@(x)f(X.u,x),x,'uniform',0));
            Pu = fPu([X.u_ind;X.u_dep])\eye(X.n);

            obj.Pu_ind = Pu*[
                eye(X.k); 
                zeros(X.m,X.k)
                ];

            obj.Pu_dep = Pu*[
                zeros(X.k,X.m); 
                eye(X.m)
                ];
        end
    end
end