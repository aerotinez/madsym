function J = jacobian(obj,qd,pose)
    arguments
        obj (1,1) Twist;
        qd (:,1) sym;
        pose (1,1) Pose = Pose();
    end
    % J = simplify(expand(pose.Adjoint*jacobian(obj.Vector,qd)));
    J = pose.Adjoint*jacobian(obj.Vector,qd);
end