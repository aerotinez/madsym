function J = jacobian(obj,q,pose)
    arguments
        obj (1,1) Twist;
        q (:,1) DynamicVariable;
        pose (1,1) Pose = obj.Pose;
    end
    V = obj.vector(pose);
    J = simplify(expand(jacobian(V,q.rate)));
end
