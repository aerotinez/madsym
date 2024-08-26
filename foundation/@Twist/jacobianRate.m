function Jd = jacobianRate(obj,q,qd,pose)
    arguments
        obj (1,1) Twist;
        q (:,1) sym;
        qd (:,1) sym;
        pose (1,1) Pose = Pose();
    end
    J = obj.jacobian(q,pose);
    f = @(j)jacobian(j,q)*qd;
    Jd = reshape(arrayfun(f,reshape(J,[],1)),size(J));
end

