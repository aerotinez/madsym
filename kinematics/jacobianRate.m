function Jd = jacobianRate(J,q,qd)
    arguments
        J sym;
        q sym;
        qd sym;
    end
    f = @(j)jacobian(j,q)*qd;
    Jd = reshape(arrayfun(f,reshape(J,[],1)),size(J));
end