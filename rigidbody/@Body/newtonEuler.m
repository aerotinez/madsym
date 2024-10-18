function eqns = newtonEuler(obj,pose)
    arguments
        obj (1,1) Body;
        pose (1,1) Pose = Pose(Frame(),Point());
    end
    T = Pose(obj.ReferenceFrame,obj.MassCenter);
    V = obj.Twist.vector();
    adV = obj.Twist.adjoint();
    Vd = obj.Twist.rateVector();
    G = obj.inertiaMatrix();
    W = Wrench(G*Vd - adV.'*G*V,T) - obj.ActiveForces;
    eqns = W.vector(pose);
end