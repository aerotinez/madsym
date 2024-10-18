function G = inertiaMatrix(obj,pose)
    arguments
        obj (1,1) Body;
        pose (1,1) Pose = Pose(obj.ReferenceFrame,obj.MassCenter);
    end
    G = blkdiag(obj.Inertia,obj.Mass.*eye(3));
    if pose ~= Pose(obj.ReferenceFrame,obj.MassCenter)
        Adbw = Pose(obj.ReferenceFrame,obj.MassCenter).inv().adjoint();
        Adwa = pose().adjoint;
        Ad = simplify(expand(Adwa*Adbw));
        G = Ad.'*G*Ad;
    end
end