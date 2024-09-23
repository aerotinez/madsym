function Jd = jacobianRate(obj,q,pose)
    arguments
        obj (1,1) Twist;
        q (:,1) DynamicVariable;
        pose (1,1) Pose = obj.Pose;
    end
    J = obj.jacobian(q,pose);
    f = @(j)jacobian(j,q.state)*q.rate;
    Jd = reshape(arrayfun(f,reshape(J,[],1)),size(J));
end