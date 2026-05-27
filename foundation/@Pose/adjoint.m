function Ad = adjoint(obj,T)
    arguments
        obj (1,1) Pose;
        T (1,1) Pose = Pose();
    end
    Ad = T.inv().Adjoint*obj.Adjoint;
end