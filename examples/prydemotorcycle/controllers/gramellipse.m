function [F,V] = gramellipse(W,ns)
    arguments
        W double;
        ns (1,1) double = 32;
    end
    
    [U,D] = eig(W);
    [d,idx] = sort(diag(D),'descend'); 
    U = U(:,idx); 
    D = diag(d);
    
    if det(U) < 0
        U(:,end) = -U(:,end);
    end
    
    G = sqrt(D);
    T = U*G*U.';

    [Xs,Ys,Zs] = sphere(ns);
    [F,Vs] = surf2patch(Xs,Ys,Zs,'triangles');

    V = Vs*T';

    if size(W,1) == 2
        V(:,3) = 0;
    end
end
