function Jd = jacobianRate(obj,qd,N)
    arguments
        obj (1,1) Frame;
        qd (:,1) sym;
        N (1,1) Frame;
    end
    Jd = simplify(expand(diff(obj.jacobian(qd,N))));
end