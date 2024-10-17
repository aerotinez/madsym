function K = kineticEnergy(obj,eomk)
    arguments
        obj (1,1) Body;
        eomk (:,1) KinematicEquations = KinematicEquations.empty(0,1);
    end
    V = obj.Twist.vector();
    if ~isempty(eomk)
        V = obj.Twist.reformulate(eomk).vector();
    end
    G = blkdiag(obj.Inertia,obj.Mass.*eye(3));
    K = (1/2).*V.'*G*V;
end