 function Jd = jacobianRate(obj,qd,pose)
    arguments
        obj (1,1) Twist;
        qd (:,1) sym;
        pose (1,1) Pose = Pose();
    end
    Jd = simplify(expand(diff(obj.jacobian(qd,pose),sym('t'))));
end