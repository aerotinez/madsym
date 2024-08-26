function Qi = inertialForces(obj)
    arguments
        obj (1,1) Body
    end
    G = blkdiag(obj.Inertia,obj.Mass*eye(3));
    V = obj.Twist.vector();
    ad = obj.Twist.adjoint();
    Vd = obj.Twist.rateVector();
    Qi = Wrench(G*Vd - ad.'*G*V,Pose(obj.ReferenceFrame,obj.MassCenter));
end