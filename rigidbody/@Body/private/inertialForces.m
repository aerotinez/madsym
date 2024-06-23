function Qi = inertialForces(obj)
    arguments
        obj (1,1) Body
    end
    G = blkdiag(obj.Inertia,obj.Mass*eye(3));
    V = obj.Twist.Vector;
    ad = obj.Twist.Adjoint;
    Vd = obj.Twist.Rate;
    Qi = G*Vd - ad.'*G*V;
end