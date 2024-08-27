function J = jacobian(obj,q,pose)
    arguments
        obj (1,1) Twist;
        q (:,1) sym;
        pose (1,1) Pose = obj.Pose;
    end
    t = sym('t');
    V = obj.vector(pose);
    qd = diff(q,t);
    J = jacobian(V,qd);
end
