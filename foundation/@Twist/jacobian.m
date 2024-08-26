function J = jacobian(obj,q,pose)
    arguments
        obj (1,1) Twist;
        q (:,1) sym;
        pose (1,1) Pose = Pose();
    end
    t = sym('t');
    J = jacobian(obj.vector(pose),diff(q,t));
end