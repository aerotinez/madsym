function eqns = newtonEuler(obj,pose,eomk)
    arguments
        obj (1,1) Body;
        pose (1,1) Pose = Pose(Frame(),Point());
        eomk (:,1) KinematicEquations = KinematicEquations.empty(0,1);
    end
    T = Pose(obj.ReferenceFrame,obj.MassCenter);
    twist = obj.Twist;
    Wa = obj.ActiveForces.vector(T);
    if ~isempty(eomk)
        twist = obj.Twist.reformulate(eomk);
        Wa = subs(Wa,eomk.States.rate,eomk.ForcingVector);
    end
    V = twist.vector();
    adV = twist.adjoint();
    Vd = twist.rateVector();
    G = obj.inertiaMatrix();
    W = Wrench(G*Vd - adV.'*G*V - Wa,T);
    eqns = W.vector(pose);
end