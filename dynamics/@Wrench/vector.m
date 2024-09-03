function W = vector(obj,T)
    arguments
        obj (1,1) Wrench;
        T (1,1) Pose = Pose();
    end
    W = simplify(expand(T.adjoint().'*obj.Vector));
end