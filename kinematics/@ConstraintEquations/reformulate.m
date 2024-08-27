function eomc = reformulate(obj,eomk)
    arguments
        obj (1,1) ConstraintEquations
        eomk (1,1) KinematicEquations
    end
    q = eomk.States;
    eomc = obj;
    J = obj.Jacobian;
    A = J*eomk.Jacobian;
    fJd = @(a)jacobian(a,q)*eomk.ForcingVector;
    Jd = reshape(arrayfun(fJd,reshape(J,[],1)),size(J));
    Ad = Jd*eomk.Jacobian + obj.Jacobian*eomk.JacobianRate;
    u = eomk.Inputs;
    eomc.Acceleration = MotionEquations(u,A,-Ad*u);
    eomc.Velocity = MotionEquations(u,A,zeros(size(eomc.Acceleration,1)));
    eomc.Jacobian = A;
    eomc.JacobianRate = Ad;
end