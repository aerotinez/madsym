function kinematics(obj,equations)
    arguments
        obj (1,1) GibbsAppell;
        equations (:,1) sym;
    end
    obj.validateKinematicEquations(equations);
    u = obj.States.Speeds.Independent;
    eomk = [
        u - equations;
        obj.Constraints.Nonholonomic
        ];
    q = obj.States.Coordinates.All;
    J = KinematicJacobian(eomk,diff(q));
    M = eye(obj.States.n,'sym');
    f = J.u(:,1:obj.States.k)*u;
    obj.Kinematics = MotionEquations(q,M,f,obj.Inputs);
end